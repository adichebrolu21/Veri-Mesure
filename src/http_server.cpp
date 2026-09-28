#include "verimeasure/http_server.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <utility>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
#define VM_INVALID_SOCKET INVALID_SOCKET
#define VM_CLOSE_SOCKET closesocket
#define VM_SEND_FLAGS 0
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
#define VM_INVALID_SOCKET (-1)
#define VM_CLOSE_SOCKET ::close
// A write to a connection the browser has already closed raises SIGPIPE on
// Linux, and SIGPIPE's default action ends the process: closing the tab that
// was showing the live feed would take the whole station down with it. Asked
// not to, send() reports the closed connection as an error instead, which is
// what Windows does anyway and what sendAll already handles.
#ifdef MSG_NOSIGNAL
#define VM_SEND_FLAGS MSG_NOSIGNAL
#else
#define VM_SEND_FLAGS 0
#endif
#endif

namespace verimeasure {
namespace {

#ifdef _WIN32
struct WinsockGuard {
    WinsockGuard() {
        WSADATA data;
        WSAStartup(MAKEWORD(2, 2), &data);
    }
    ~WinsockGuard() { WSACleanup(); }
};
WinsockGuard g_winsock;
#endif

const char* reasonPhrase(int code) {
    switch (code) {
        case 200: return "OK";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        default: return "OK";
    }
}

bool readLine(socket_t sock, std::string& buffer, std::string& line) {
    while (true) {
        const std::size_t pos = buffer.find("\r\n");
        if (pos != std::string::npos) {
            line = buffer.substr(0, pos);
            buffer.erase(0, pos + 2);
            return true;
        }
        char chunk[4096];
        const int n = static_cast<int>(recv(sock, chunk, sizeof(chunk), 0));
        if (n <= 0) return false;
        buffer.append(chunk, static_cast<std::size_t>(n));
    }
}

}  // namespace

std::string urlDecode(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t k = 0; k < text.size(); ++k) {
        if (text[k] == '+') {
            out += ' ';
        } else if (text[k] == '%' && k + 2 < text.size()) {
            const std::string hex = text.substr(k + 1, 2);
            out += static_cast<char>(std::strtol(hex.c_str(), nullptr, 16));
            k += 2;
        } else {
            out += text[k];
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// HttpConnection
// ---------------------------------------------------------------------------

bool HttpConnection::sendAll(const char* data, std::size_t n) {
    const socket_t sock = static_cast<socket_t>(socket_);
    std::size_t sent = 0;
    while (sent < n) {
        const int wrote =
            static_cast<int>(send(sock, data + sent, static_cast<int>(n - sent), VM_SEND_FLAGS));
        if (wrote <= 0) {
            alive_ = false;
            return false;
        }
        sent += static_cast<std::size_t>(wrote);
    }
    return true;
}

void HttpConnection::sendResponse(int code, const std::string& contentType,
                                  const std::string& body) {
    char header[512];
    const int n = std::snprintf(header, sizeof(header),
                                "HTTP/1.1 %d %s\r\n"
                                "Content-Type: %s\r\n"
                                "Content-Length: %zu\r\n"
                                "Cache-Control: no-store\r\n"
                                "\r\n",
                                code, reasonPhrase(code), contentType.c_str(), body.size());
    if (!sendAll(header, static_cast<std::size_t>(n))) return;
    sendAll(body.data(), body.size());
}

void HttpConnection::beginMjpeg() {
    static const char header[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-store\r\n"
        "\r\n";
    sendAll(header, sizeof(header) - 1);
}

bool HttpConnection::sendJpegPart(const std::vector<unsigned char>& jpeg) {
    char header[128];
    const int n = std::snprintf(header, sizeof(header),
                                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                                jpeg.size());
    if (!sendAll(header, static_cast<std::size_t>(n))) return false;
    if (!sendAll(reinterpret_cast<const char*>(jpeg.data()), jpeg.size())) return false;
    return sendAll("\r\n", 2);
}

// ---------------------------------------------------------------------------
// HttpServer
// ---------------------------------------------------------------------------

HttpServer::HttpServer(std::string host, int port, RequestHandler handler)
    : host_(std::move(host)), port_(port), handler_(std::move(handler)) {}

HttpServer::~HttpServer() {
    shutdown();
}

bool HttpServer::start() {
    const socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == VM_INVALID_SOCKET) {
        std::fprintf(stderr, "could not create a socket\n");
        return false;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse),
               sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<unsigned short>(port_));
    if (host_.empty() || host_ == "0.0.0.0") {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
        std::fprintf(stderr, "could not interpret host %s\n", host_.c_str());
        VM_CLOSE_SOCKET(sock);
        return false;
    }

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::fprintf(stderr, "could not bind %s:%d, is the port already in use?\n", host_.c_str(),
                     port_);
        VM_CLOSE_SOCKET(sock);
        return false;
    }
    if (listen(sock, 32) != 0) {
        std::fprintf(stderr, "could not listen on %s:%d\n", host_.c_str(), port_);
        VM_CLOSE_SOCKET(sock);
        return false;
    }

    listen_.store(static_cast<long long>(sock));
    return true;
}

void HttpServer::serveForever() {
    while (!stop_.load()) {
        sockaddr_in peer{};
#ifdef _WIN32
        int len = sizeof(peer);
#else
        socklen_t len = sizeof(peer);
#endif
        const socket_t client = accept(static_cast<socket_t>(listen_.load()),
                                       reinterpret_cast<sockaddr*>(&peer), &len);
        if (client == VM_INVALID_SOCKET) {
            if (stop_.load()) break;
            continue;
        }
        std::thread(&HttpServer::handleClient, this, static_cast<long long>(client)).detach();
    }
}

void HttpServer::shutdown() {
    if (stop_.exchange(true)) return;
    const long long sock = listen_.exchange(-1);
    if (sock != -1) {
#ifndef _WIN32
        // Closing the socket is enough to wake a thread blocked in accept() on
        // Windows, and not on Linux: there the blocked call keeps its own hold
        // on the socket and sleeps on, so Ctrl-C would leave the station
        // running with nothing listening. Shutting it down first makes that
        // accept() return at once.
        ::shutdown(static_cast<socket_t>(sock), SHUT_RDWR);
#endif
        VM_CLOSE_SOCKET(static_cast<socket_t>(sock));
    }
}

void HttpServer::handleClient(long long socketId) {
    const socket_t sock = static_cast<socket_t>(socketId);

#ifndef _WIN32
    int nodelay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
#endif

    std::string buffer;
    HttpConnection conn(socketId);

    while (conn.alive() && !stop_.load()) {
        std::string requestLine;
        if (!readLine(sock, buffer, requestLine)) break;
        if (requestLine.empty()) continue;

        std::string method, target;
        {
            const std::size_t a = requestLine.find(' ');
            if (a == std::string::npos) break;
            const std::size_t b = requestLine.find(' ', a + 1);
            method = requestLine.substr(0, a);
            target = requestLine.substr(a + 1, b == std::string::npos ? std::string::npos : b - a - 1);
        }

        long long contentLength = 0;
        bool close = false;
        while (true) {
            std::string header;
            if (!readLine(sock, buffer, header)) {
                conn.markClosed();
                break;
            }
            if (header.empty()) break;
            const std::size_t colon = header.find(':');
            if (colon == std::string::npos) continue;
            std::string name = header.substr(0, colon);
            std::string value = header.substr(colon + 1);
            while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
                value.erase(value.begin());
            }
            for (char& c : name) c = static_cast<char>(std::tolower(c));
            if (name == "content-length") contentLength = std::strtoll(value.c_str(), nullptr, 10);
            if (name == "connection") {
                std::string lowered = value;
                for (char& c : lowered) c = static_cast<char>(std::tolower(c));
                if (lowered.find("close") != std::string::npos) close = true;
            }
        }
        if (!conn.alive()) break;

        // The commands carry their arguments in the query string, so any body is
        // read and discarded to keep the connection in step.
        while (contentLength > 0) {
            if (!buffer.empty()) {
                const std::size_t take =
                    std::min<std::size_t>(buffer.size(), static_cast<std::size_t>(contentLength));
                buffer.erase(0, take);
                contentLength -= static_cast<long long>(take);
                continue;
            }
            char chunk[4096];
            const int n = static_cast<int>(recv(sock, chunk, sizeof(chunk), 0));
            if (n <= 0) {
                conn.markClosed();
                break;
            }
            contentLength -= n;
        }
        if (!conn.alive()) break;

        std::string path = target;
        std::map<std::string, std::string> query;
        const std::size_t q = target.find('?');
        if (q != std::string::npos) {
            path = target.substr(0, q);
            std::string rest = target.substr(q + 1);
            std::size_t start = 0;
            while (start <= rest.size()) {
                const std::size_t amp = rest.find('&', start);
                const std::string pair =
                    rest.substr(start, amp == std::string::npos ? std::string::npos : amp - start);
                if (!pair.empty()) {
                    const std::size_t eq = pair.find('=');
                    if (eq == std::string::npos) {
                        query[urlDecode(pair)] = std::string();
                    } else {
                        query[urlDecode(pair.substr(0, eq))] = urlDecode(pair.substr(eq + 1));
                    }
                }
                if (amp == std::string::npos) break;
                start = amp + 1;
            }
        }

        handler_(method, path, query, conn);
        if (close) break;
    }

    VM_CLOSE_SOCKET(sock);
}

}  // namespace verimeasure
