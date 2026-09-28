// A small threaded HTTP/1.1 server, enough to serve one page, two JSON
// endpoints, a command endpoint and an MJPEG stream. This replaces Python's
// ThreadingHTTPServer; the routes and their behaviour are unchanged.

#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace verimeasure {

class HttpConnection {
public:
    explicit HttpConnection(long long socket) : socket_(socket) {}

    // Single response with Content-Length and no-store caching.
    void sendResponse(int code, const std::string& contentType, const std::string& body);

    // Multipart MJPEG stream. beginMjpeg() writes the headers; sendJpegPart()
    // returns false once the client has gone away.
    void beginMjpeg();
    bool sendJpegPart(const std::vector<unsigned char>& jpeg);

    bool alive() const { return alive_; }
    void markClosed() { alive_ = false; }

private:
    bool sendAll(const char* data, std::size_t n);

    long long socket_;
    bool alive_ = true;
};

using RequestHandler = std::function<void(const std::string& method, const std::string& path,
                                          const std::map<std::string, std::string>& query,
                                          HttpConnection& conn)>;

class HttpServer {
public:
    HttpServer(std::string host, int port, RequestHandler handler);
    ~HttpServer();

    // Binds and listens. Returns false with the reason on stderr if the port is
    // in use or the address is unavailable.
    bool start();

    // Accepts until shutdown() is called from another thread.
    void serveForever();
    void shutdown();

private:
    void handleClient(long long socket);

    std::string host_;
    int port_;
    RequestHandler handler_;
    // Written by shutdown(), which may run on another thread or in a signal
    // handler, while serveForever() is reading it.
    std::atomic<long long> listen_{-1};
    std::atomic<bool> stop_{false};
};

// Percent-decoding, with '+' meaning a space, as parse_qs does.
std::string urlDecode(const std::string& text);

}  // namespace verimeasure
