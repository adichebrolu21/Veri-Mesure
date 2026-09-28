#include "verimeasure/segmentation.hpp"

#include <algorithm>
#include <cmath>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/numeric.hpp"

namespace verimeasure {

cv::Mat removeSpeckle(const cv::Mat& mask, double minAreaPx) {
    if (minAreaPx < 1.0) return mask;

    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    if (n <= 2) return mask;

    std::vector<unsigned char> keep(static_cast<std::size_t>(n), 0);
    for (int i = 1; i < n; ++i) {
        keep[static_cast<std::size_t>(i)] =
            (stats.at<int>(i, cv::CC_STAT_AREA) >= minAreaPx) ? 255 : 0;
    }
    keep[0] = 0;

    cv::Mat out(mask.size(), CV_8UC1);
    for (int y = 0; y < mask.rows; ++y) {
        const int* lab = labels.ptr<int>(y);
        unsigned char* d = out.ptr<unsigned char>(y);
        for (int x = 0; x < mask.cols; ++x) d[x] = keep[static_cast<std::size_t>(lab[x])];
    }
    return out;
}

void segmentObject(const cv::Mat& roiBgr, const cv::Mat& backgroundBgr, const Config& cfg,
                   double gain, int bridgePx, double speckleMinPx, cv::Mat& mask,
                   double& changedFraction, cv::Mat& score, bool darkerGarment,
                   double blackCeiling) {
    CV_Assert(roiBgr.type() == CV_8UC3 && backgroundBgr.type() == CV_8UC3);
    CV_Assert(roiBgr.size() == backgroundBgr.size());

    const int rows = roiBgr.rows;
    const int cols = roiBgr.cols;
    const float gainf = static_cast<float>(gain);
    const float chromaThr = static_cast<float>(cfg.chroma_angle_threshold);
    const float ratioHigh = static_cast<float>(cfg.intensity_ratio_high);
    const float ratioLow = static_cast<float>(cfg.intensity_ratio_low);
    const float shadowSpread = static_cast<float>(cfg.shadow_uniformity);
    const float shadowFloor = static_cast<float>(cfg.shadow_ratio_floor);

    cv::Mat raw(rows, cols, CV_8UC1);
    score.create(rows, cols, CV_32FC1);
    // Where the texture evidence was confident enough to call a shadow, and
    // where the picture is merely darker than it was. The second pass below
    // needs both.
    cv::Mat darkConf(rows, cols, CV_32FC1);
    cv::Mat litConf(rows, cols, CV_32FC1);
    cv::Mat darkerThanBg(rows, cols, CV_32FC1);
    cv::Mat brighterThanBg(rows, cols, CV_32FC1);

    // How much of the surface's own texture is still showing.
    //
    // Colour cannot finish this argument. A black garment lying over a printed
    // grid line is the same colour as that grid line with almost all the light
    // taken away, which is precisely what a shadow is: measured against this
    // mat, black-over-line gives a colour angle of 0.003 and a channel spread
    // of 0.026, and a real shadow over the same line gives 0.0002 and 0.006.
    // Nothing separates those two numbers that is not also noise.
    //
    // What separates them is the grid. A shadow is the mat with less light on
    // it, so the line is still there, dimmer in the same proportion as
    // everything around it. A garment is the mat with a garment on top of it
    // and the line is gone. So the local spread of the picture is compared with
    // the local spread of the background scaled by the same factor the
    // brightness fell by: a shadow keeps its share of it, an opaque thing of
    // any colour keeps none.
    //
    // The window has to fit inside the shadow it is being asked about.
    //
    // That is the constraint that decides this, and it points the opposite way
    // to the obvious one. A wide window is tempting because the mat always has
    // something printed somewhere in it to judge by, but a shadow beside a
    // folded garment is only 25 to 60 px across, and a window wider than that
    // is mostly looking at the garment on one side and clear mat on the other.
    // Measured in a 40 px shadow, a 97 px window reports a local spread of 100
    // - the garment, not the shadow - and a correlation of 0.04, so it can
    // never recognise the thing it is standing in. At 21 px it reports 0.65 and
    // does.
    //
    // Nothing is lost by being narrow. What the evidence buys is never
    // completeness - even at its best it speaks for barely half the shadow,
    // because between two printed lines there is nothing to speak about - it is
    // certainty: swept across window sizes from 9 px to 41 px, the fraction of
    // a pale garment this test mistakes for shadow is 0.0% at every one of
    // them. Sparse and certain is the right shape of evidence for a region
    // vote to finish, and that is what happens below.
    cv::Mat sdCur, sdBg, ncc;
    // With a garment near black: how dark the picture is, smoothed just
    // enough that the sensor's noise in cloth this dark does not decide it.
    const bool black = darkerGarment && blackCeiling > 0.0;
    const float ceilingF = static_cast<float>(blackCeiling);
    cv::Mat curSmooth;
    {
        cv::Mat curGray, bgGray, curF, bgF;
        cv::cvtColor(roiBgr, curGray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(backgroundBgr, bgGray, cv::COLOR_BGR2GRAY);
        curGray.convertTo(curF, CV_32F);
        bgGray.convertTo(bgF, CV_32F);
        if (black) cv::GaussianBlur(curF, curSmooth, cv::Size(0, 0), 1.0);

        auto textureAt = [&](int w, cv::Mat& sdC, cv::Mat& sdB, cv::Mat& corr) {
            const cv::Size k(std::max(3, w | 1), std::max(3, w | 1));
            cv::Mat mCur, mBg, m2;
            cv::boxFilter(curF, mCur, CV_32F, k);
            cv::boxFilter(curF.mul(curF), m2, CV_32F, k);
            cv::sqrt(cv::max(m2 - mCur.mul(mCur), 0.0f), sdC);
            cv::boxFilter(bgF, mBg, CV_32F, k);
            cv::boxFilter(bgF.mul(bgF), m2, CV_32F, k);
            cv::sqrt(cv::max(m2 - mBg.mul(mBg), 0.0f), sdB);
            cv::boxFilter(curF.mul(bgF), m2, CV_32F, k);
            cv::divide(m2 - mCur.mul(mBg), sdC.mul(sdB) + 1e-3f, corr);
        };

        textureAt(cfg.shadow_texture_window_px, sdCur, sdBg, ncc);
    }
    const float textureKeep = static_cast<float>(cfg.shadow_texture_keep);
    const float textureCorr = static_cast<float>(cfg.shadow_texture_correlation);
    const bool handlesGlow = cfg.segment_handles_glow;
    const int shallowEvidence = cfg.shadow_shallow_needs_evidence;

    for (int y = 0; y < rows; ++y) {
        const unsigned char* cur = roiBgr.ptr<unsigned char>(y);
        const unsigned char* bg = backgroundBgr.ptr<unsigned char>(y);
        unsigned char* m = raw.ptr<unsigned char>(y);
        float* s = score.ptr<float>(y);
        const float* sdC = sdCur.ptr<float>(y);
        const float* sdB = sdBg.ptr<float>(y);
        const float* corr = ncc.ptr<float>(y);
        float* dconf = darkConf.ptr<float>(y);
        float* lconf = litConf.ptr<float>(y);
        float* dark = darkerThanBg.ptr<float>(y);
        float* lit = brighterThanBg.ptr<float>(y);
        const float* sm = black ? curSmooth.ptr<float>(y) : nullptr;

        for (int x = 0; x < cols; ++x) {
            const int base = x * 3;

            float dot = 0.0f;
            float nCurSq = 0.0f;
            float nBgSq = 0.0f;
            float ratioMax = 0.0f;
            float ratioMin = 0.0f;
            float diffSq = 0.0f;

            for (int i = 0; i < 3; ++i) {
                const float rawCur = static_cast<float>(cur[base + i]);
                const float rawBg = static_cast<float>(bg[base + i]);

                const float c = rawCur + 1.0f;
                const float b = rawBg * gainf + 1.0f;

                dot += c * b;
                nCurSq += c * c;
                nBgSq += b * b;

                const float r = c / b;
                if (i == 0) {
                    ratioMax = r;
                    ratioMin = r;
                } else {
                    ratioMax = std::max(ratioMax, r);
                    ratioMin = std::min(ratioMin, r);
                }

                // Foreground evidence for the sub-pixel edge fit uses the raw
                // difference, with no offset, so the printed grid cancels.
                const float d = rawCur - rawBg * gainf;
                diffSq += d * d;
            }

            const float nCur = std::sqrt(nCurSq);
            const float nBg = std::sqrt(nBgSq);
            const float chromaChange = 1.0f - dot / (nCur * nBg + 1e-6f);
            const float ratio = nCur / (nBg + 1e-6f);
            const float spread = ratioMax - ratioMin;

            // A shadow is the same light, less of it: every channel scales by
            // the same factor, so the spread across the channel ratios stays
            // small and the colour direction does not move at all.
            const bool uniformlyDarker = (ratio < 0.95f) && (spread < shadowSpread);

            // The floor of 0.25 is what a diffuse room could produce. A lamp
            // cannot.
            //
            // Under window light a cast shadow is filled in from every other
            // direction and rarely falls below a quarter of the lit surface, so
            // a quarter was a safe place to stop believing in shadows and start
            // believing in genuinely dark objects. A single bright source has no
            // such fill: the shadow a folded garment throws beside itself goes
            // to a tenth of the background or less. That is below the floor, so
            // it was not a shadow; it is below intensity_ratio_low, so it was
            // foreground; and it is attached to the garment, so it became part
            // of the garment. The outline runs along the outside of the shadow
            // and every measurement across it reads long by the width of it.
            //
            // Below the floor the colour direction has to agree as well. That is
            // what keeps a genuinely dark object out of this: a black garment on
            // a dark green mat is not the mat with less light on it, and the
            // angle between the two colours says so, whereas a shadow of any
            // depth leaves that angle at zero.
            // Below 2 the background has no texture to have kept or lost -
            // bare mat between two printed lines - and an absence of evidence
            // is not evidence of a shadow.
            const bool textureSurvives =
                (sdB[x] > 2.0f) && (sdC[x] > textureKeep * ratio * sdB[x]);
            // The mat's pattern is not merely still as strong as it should be,
            // it is still in the same places. That is what an opaque thing
            // cannot fake and a shadow cannot help doing.
            const bool patternSurvives = textureSurvives && (corr[x] > textureCorr);

            const bool deepShadow = uniformlyDarker && (ratio > shadowFloor) &&
                                    (chromaChange <= chromaThr) && textureSurvives;
            // Darker, in any colour, with the surface underneath still showing
            // through it. This is the clause that catches a shadow filled by
            // room light, which no colour test here will ever agree is one.
            const bool shadedButShowing =
                (ratio < 0.95f) && (ratio > shadowFloor) && patternSurvives;

            // Brighter, with the surface underneath still showing through it.
            //
            // The mat beside a garment is not only darkened, it is also lit.
            // A bright cloth under a bright lamp throws light back onto the
            // surface next to it, and that glow is as real as any shadow and
            // rather larger: measured on this bench, the mat in the 40 px below
            // a hem comes back at twice the brightness of the same mat a
            // hand's width away, with 6.5% of it clipped white, against 0.03%
            // further out. Twice the brightness is a ratio of 1.9 against an
            // intensity_ratio_high of 1.25, so every pixel of it was
            // foreground, and the outline ran 52 px - 7 mm - out into the glow.
            //
            // It scalloped along the printed grid on the way, which is what
            // gave it away: the squares and the lines brighten by different
            // amounts and clip at different points, so the glow crosses the
            // threshold in the squares and not on the lines, and the boundary
            // dips into every square with the pitch of the grid.
            //
            // None of that needs a test of its own. A glow is the same thing a
            // shadow is - this surface, under different light - and the
            // evidence that settles it is the same evidence: the mat's printed
            // pattern is still there, in the same places, which no opaque cloth
            // laid on top of it can imitate.
            const bool litButShowing = handlesGlow && (ratio > 1.05f) && patternSurvives;

            // A shallow shadow on its colour alone - unless the bench asks
            // for evidence as well (shadow_shallow_needs_evidence).
            bool shallowShadow = uniformlyDarker && ratio > 0.25f;
            if (shallowEvidence == 1) {
                shallowShadow = shallowShadow && patternSurvives;
            } else if (shallowEvidence == 2) {
                shallowShadow = shallowShadow && (chromaChange <= 0.5f * chromaThr);
            }

            // A garment near black is everything as dark as it is. Nothing on
            // the base gets that dark beside it on this bench - its shadow
            // there is half the base's light or more - so a pixel this dark is
            // cloth whatever its colour or texture would otherwise say.
            const bool asDarkAsCloth = black && sm[x] <= ceilingF;

            const bool looksLikeSurface = !asDarkAsCloth && (shallowShadow || deepShadow ||
                                                             shadedButShowing || litButShowing);

            const bool foreground =
                asDarkAsCloth ||
                (((chromaChange > chromaThr) || (ratio > ratioHigh) || (ratio < ratioLow)) &&
                 !looksLikeSurface);

            m[x] = foreground ? 255 : 0;
            s[x] = looksLikeSurface ? 0.0f : std::sqrt(diffSq);
            // The two populations the region votes are taken over. A pixel
            // that matches its background was never in question and casts no
            // vote in either.
            const bool isDark = ratio < 0.95f;
            const bool isLit = ratio > 1.05f;
            dark[x] = isDark ? 1.0f : 0.0f;
            lit[x] = isLit ? 1.0f : 0.0f;
            dconf[x] = (looksLikeSurface && isDark) ? 1.0f : 0.0f;
            lconf[x] = (looksLikeSurface && isLit) ? 1.0f : 0.0f;
        }
    }

    // A shadow does not come in stripes.
    //
    // The tests above can only speak where the mat has something printed for
    // them to look at, and between two grid lines it has not: those pixels get
    // no verdict, fall through to being foreground, and what is left is a
    // shadow correctly rejected along every line and still accepted in the
    // gaps between them. The mask keeps its full width on the strength of the
    // gaps alone, which is the whole of the error, because an outline is drawn
    // round the outermost pixel and does not care that the ones beside it were
    // dropped.
    //
    // But a shadow is a region, not a scattering of pixels. Where the evidence
    // did speak it was confident, so the question for a pixel it could not
    // judge is what its neighbours were: darker than the background, and inside
    // a neighbourhood the texture has already shown to be shadow, is shadow. A
    // box filter over the verdicts is that neighbourhood vote, and it costs the
    // same as the ones already being run.
    //
    // A dark garment is not at risk from this. It is darker, but nothing around
    // it correlates with the mat, so the vote it sits in is empty.
    {
        // The vote counts only the pixels entitled to one.
        //
        // Sizing this window is a trap if it is averaged over its own area. It
        // has to be wider than the gap between two printed lines, or a pixel in
        // the middle of a gap hears from nobody and the shadow survives in
        // stripes 60 px across - and a contour is drawn round the outermost
        // pixel, so surviving in stripes holds the outline out exactly as far
        // as surviving everywhere. But it also has to not be diluted by the
        // garment beside it, and at this mat's 72 px pitch a window wide enough
        // for the first is far too wide for the second.
        //
        // The two stop fighting once the denominator is the number of darker
        // pixels in the window rather than the number of pixels in it. The
        // garment is not darker, the clear mat beside the shadow is not darker,
        // and neither of them is asked. What the vote then reports is: of the
        // things around here that are in shadow-or-object territory at all,
        // what share did the texture positively identify as shadow? That
        // question has the same answer whether the window is full of shadow or
        // has a garment across two thirds of it, so the window can be as wide
        // as the grid needs it to be.
        // Darker and brighter are counted apart, and they have to be.
        //
        // Pooling them looks tidier and is wrong: the garment is brighter than
        // the mat, so putting brightness into the same population as shadow
        // lets a garment beside a shadow dilute the shadow's own vote. Measured,
        // that cost 2 to 4 mm on the traced outline of cases that were clean
        // before, which the side-fitting then averaged away - so the number
        // still looked right while the line that was drawn was not. A shadow is
        // asked about among the other darker pixels near it, and a glow among
        // the other brighter ones.
        // Only shadows get a region vote. A glow does not, and must not.
        //
        // The vote works by letting pixels with no evidence of their own be
        // spoken for by their neighbours, and its safety rests entirely on the
        // garment being outside the population doing the speaking. For shadow
        // that holds: cloth laid on a mat is not darker than the mat, so a vote
        // among darker pixels cannot reach it. For glow it does not hold at
        // all - a pale garment is the brightest thing in the picture, so it
        // sits in the very population being polled, and a vote carried by the
        // bounce beside it removes the garment with it. On this bench that took
        // 33 mm off a 225 mm sleeve, which is worse than the 7 mm of glow it
        // was there to remove.
        //
        // So a glow is only ever recognised a pixel at a time, on that pixel's
        // own evidence, which requires the mat's pattern to be positively
        // present where it is standing. Cloth cannot produce that, so nothing
        // the vote could have added is worth what it can take away.
        cv::Mat darkConfSum, darkSum, darkVote;
        const int w = std::max(3, cfg.shadow_region_window_px | 1);
        const cv::Size k2(w, w);
        cv::boxFilter(darkConf, darkConfSum, CV_32F, k2, cv::Point(-1, -1), false);
        cv::boxFilter(darkerThanBg, darkSum, CV_32F, k2, cv::Point(-1, -1), false);
        cv::divide(darkConfSum, cv::max(darkSum, 1.0f), darkVote);

        // The glow gets a vote too, but a short-sighted one.
        //
        // It needs one at all because the brightest part of a glow is the part
        // that clips, and a clipped pixel is flat white with no pattern left in
        // it to recognise - so the band immediately beside the garment, which
        // is the part that moves the measurement, is exactly the part that
        // cannot speak for itself. But its reach is what makes it dangerous,
        // because a pale garment is in the population it polls. So it is given
        // a fraction of the shadow vote's window: far enough to carry across a
        // clipped band, never far enough to take a bite out of cloth. At this
        // width the worst it can cost a garment is half the window - under two
        // millimetres - and only where that garment is itself clipped, since
        // anywhere else the guard below has already stopped it.
        cv::Mat litConfSum, litSum, litVote;
        const int wg = std::max(3, cfg.glow_region_window_px | 1);
        const cv::Size k3(wg, wg);
        cv::boxFilter(litConf, litConfSum, CV_32F, k3, cv::Point(-1, -1), false);
        cv::boxFilter(brighterThanBg, litSum, CV_32F, k3, cv::Point(-1, -1), false);
        cv::divide(litConfSum, cv::max(litSum, 1.0f), litVote);

        const float need = static_cast<float>(cfg.shadow_region_fraction);
        for (int y = 0; y < rows; ++y) {
            unsigned char* m = raw.ptr<unsigned char>(y);
            float* s = score.ptr<float>(y);
            const float* dv = darkVote.ptr<float>(y);
            const float* lv = litVote.ptr<float>(y);
            const float* dark = darkerThanBg.ptr<float>(y);
            const float* lit = brighterThanBg.ptr<float>(y);
            const float* sdB = sdBg.ptr<float>(y);
            const float* corr = ncc.ptr<float>(y);
            const unsigned char* cur = roiBgr.ptr<unsigned char>(y);
            const float* sm = black ? curSmooth.ptr<float>(y) : nullptr;
            for (int x = 0; x < cols; ++x) {
                // A vote fills a silence; it does not overrule a witness. Where
                // the mat has a pattern and this pixel is measurably not it,
                // the pixel has said something about itself and the neighbours
                // do not get to contradict it.
                //
                // Clipped white is not a witness. A pixel at 255 has had its
                // pattern taken away by the sensor, not by anything on the
                // bench, so the correlation there is low for a reason that says
                // nothing about what is underneath - and the brightest part of
                // a glow, the part beside the garment that actually moves the
                // measurement, is precisely the part that clips. Counting that
                // as testimony against the vote is how a glow keeps 5 mm of
                // itself.
                const int base = x * 3;
                const bool clipped = cur[base] >= 254 || cur[base + 1] >= 254 ||
                                     cur[base + 2] >= 254;
                // Only while a glow is being recognised. The shadow vote has
                // no need of this guard: cloth is not darker than the mat it
                // lies on, so a vote among darker pixels cannot reach a garment
                // to be stopped from reaching it, and making it stop costs
                // about 0.7 mm of the traced outline for nothing.
                const bool contrary =
                    handlesGlow && !clipped && sdB[x] > 2.0f && corr[x] < 0.25f;
                // ...except that cloth darker than the base does exist: a black
                // shirt on the blue cloth reads a seventh of the cloth, and the
                // vote carried by the shadow beside it took 5 to 7 mm off every
                // edge it had and bit into its corners. So with a garment
                // darker than the base, the same witness stands against the
                // shadow vote too. A shadow's own pixels keep the base's
                // pattern and are not witnesses, so the shadow still goes.
                //
                // A garment near black needs no witness and is not given one.
                // What stands against the vote there is darkness: a pixel as
                // dark as the cloth is cloth. A missing pattern is not enough,
                // because the shadow of a black shirt across a printed line,
                // or a line the base has crept off, is missing its pattern too
                // - and those were the lines the outline ran out along.
                const bool witnessed = black ? sm[x] <= ceilingF
                                             : darkerGarment && sdB[x] > 2.0f && corr[x] < 0.25f;
                const bool voted = (dark[x] > 0.5f && dv[x] > need) ||
                                   (handlesGlow && lit[x] > 0.5f && lv[x] > need);
                if (!contrary && !witnessed && voted) {
                    m[x] = 0;
                    // The edge fit is given the same verdict the mask was.
                    //
                    // It used to get the raw difference whatever the pixel
                    // turned out to be, and a shadow is a large raw difference,
                    // so even with the shadow kept out of the mask the
                    // refinement searching outward along the normal still had a
                    // ridge of evidence sitting in the shadow to be pulled
                    // toward. Zeroed, what it sees across a garment edge is one
                    // step and nothing else: full evidence on the cloth, none
                    // on the surface beside it, lit or shaded.
                    s[x] = 0.0f;
                }
            }
        }
    }

    const cv::Mat k =
        cv::Mat::ones(cfg.morph_kernel_size, cfg.morph_kernel_size, CV_8U);
    cv::morphologyEx(raw, mask, cv::MORPH_OPEN, k);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);

    if (speckleMinPx > 0.0) {
        mask = removeSpeckle(mask, speckleMinPx);
    }

    if (bridgePx >= 3) {
        const int b = bridgePx | 1;
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(b, b)));
    }

    // Sized off the printed pitch, so this is a physical width and stays one on
    // the decimated pass too. Taken from the bridge when the caller supplied
    // one, since that is already a fraction of the pitch; a caller that asked
    // for no bridging has said nothing about boundary quality, so it falls back
    // to the same pitch estimate the bridge itself would have used.
    const int filamentScale = bridgePx > 0 ? bridgePx : cfg.mask_bridge_px_fallback;
    const int filamentPx =
        static_cast<int>(std::lround(filamentScale * cfg.outline_filament_fraction));

    // Take the whiskers off.
    //
    // What is left on the boundary at this point is thin: filaments a few
    // pixels wide reaching out onto the mat, which the contour walks down one
    // side of and back up the other, so the drawn outline sprouts a row of thin
    // teardrops off an otherwise clean edge. They come from the printed lines
    // lighting up beside a garment that is clipping, and they are not where the
    // cloth is: measured through this pipeline they put 3.8 mm on a 114 mm
    // reading.
    //
    // A plain opening, and NOT an opening by reconstruction, which is the
    // tempting answer and is wrong here. Reconstruction regrows every component
    // that keeps a surviving seed, and a whisker is attached to the garment, so
    // the geodesic dilation flows straight back out into it and puts it back.
    // It removes detached specks, which was never the problem.
    //
    // So the opening, whose price is that it rounds convex corners by the
    // kernel radius too. That price is paid on the corner pixel alone: a fold
    // is measured from the fitted straight runs either side of it, hundreds of
    // points long, which an opening does not move at all.
    if (filamentPx >= 3) {
        const int r = filamentPx | 1;
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(r, r)));
    }

    changedFraction =
        static_cast<double>(cv::countNonZero(mask)) / static_cast<double>(mask.total());
}

double robustGain(const cv::Mat& curBgr, const cv::Mat& bgBgr) {
    const cv::Mat cur = strideSample(curBgr, 4);
    const cv::Mat bg = strideSample(bgBgr, 4);

    std::vector<double> ratios;
    ratios.reserve(static_cast<std::size_t>(cur.total()));

    for (int y = 0; y < cur.rows; ++y) {
        const unsigned char* c = cur.ptr<unsigned char>(y);
        const unsigned char* b = bg.ptr<unsigned char>(y);
        for (int x = 0; x < cur.cols; ++x) {
            const int i = x * 3;
            const float lc = static_cast<float>(c[i]) + static_cast<float>(c[i + 1]) +
                             static_cast<float>(c[i + 2]);
            float lb = static_cast<float>(b[i]) + static_cast<float>(b[i + 1]) +
                       static_cast<float>(b[i + 2]);
            if (lb < 1.0f) lb = 1.0f;
            ratios.push_back(static_cast<double>(lc / lb));
        }
    }

    const double m = median(std::move(ratios));
    return std::min(2.0, std::max(0.5, m));
}

bool matGain(const cv::Mat& curBgr, const cv::Mat& bgBgr, const cv::Mat& objectMask,
             int marginPx, double minFraction, double& gain) {
    // Filled, so a hole the segmentation left inside the garment is not taken
    // for mat, and grown, so the shadow and glow beside it are not either.
    cv::Mat covered = cv::Mat::zeros(objectMask.size(), CV_8UC1);
    {
        std::vector<std::vector<cv::Point>> outlines;
        cv::findContours(objectMask.clone(), outlines, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(covered, outlines, -1, cv::Scalar(255), cv::FILLED);
    }
    if (marginPx > 0) {
        cv::dilate(covered, covered,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                             cv::Size(2 * marginPx + 1, 2 * marginPx + 1)));
    }

    // The samples robustGain takes, in the same order and arithmetic, less
    // the ones on or beside the garment.
    const cv::Mat cur = strideSample(curBgr, 4);
    const cv::Mat bg = strideSample(bgBgr, 4);
    const cv::Mat off = strideSample(covered, 4);

    std::vector<double> ratios;
    ratios.reserve(static_cast<std::size_t>(cur.total()));
    for (int y = 0; y < cur.rows; ++y) {
        const unsigned char* c = cur.ptr<unsigned char>(y);
        const unsigned char* b = bg.ptr<unsigned char>(y);
        const unsigned char* o = off.ptr<unsigned char>(y);
        for (int x = 0; x < cur.cols; ++x) {
            if (o[x]) continue;
            const int i = x * 3;
            const float lc = static_cast<float>(c[i]) + static_cast<float>(c[i + 1]) +
                             static_cast<float>(c[i + 2]);
            float lb = static_cast<float>(b[i]) + static_cast<float>(b[i + 1]) +
                       static_cast<float>(b[i + 2]);
            if (lb < 1.0f) lb = 1.0f;
            ratios.push_back(static_cast<double>(lc / lb));
        }
    }
    const double need = std::max(1.0, minFraction * static_cast<double>(cur.total()));
    if (static_cast<double>(ratios.size()) < need) return false;
    const double m = median(std::move(ratios));
    gain = std::min(2.0, std::max(0.5, m));
    return true;
}

bool darkerThanBase(const cv::Mat& curBgr, const cv::Mat& bgBgr, const cv::Mat& mask, double gain,
                    double ratio) {
    // The samples robustGain takes, over what the mask covers.
    const cv::Mat cur = strideSample(curBgr, 4);
    const cv::Mat bg = strideSample(bgBgr, 4);
    const cv::Mat on = strideSample(mask, 4);
    std::vector<double> ratios;
    for (int y = 0; y < cur.rows; ++y) {
        const unsigned char* c = cur.ptr<unsigned char>(y);
        const unsigned char* b = bg.ptr<unsigned char>(y);
        const unsigned char* o = on.ptr<unsigned char>(y);
        for (int x = 0; x < cur.cols; ++x) {
            if (!o[x]) continue;
            const int i = x * 3;
            const double lc = static_cast<double>(c[i]) + c[i + 1] + c[i + 2];
            const double lb = std::max(1.0, (static_cast<double>(b[i]) + b[i + 1] + b[i + 2]) * gain);
            ratios.push_back(lc / lb);
        }
    }
    return !ratios.empty() && median(std::move(ratios)) < ratio;
}

double blackGarmentCeiling(const cv::Mat& curBgr, const cv::Mat& bgBgr, const cv::Mat& mask,
                           double gain, const Config& cfg) {
    if (!(cfg.dark_garment_black_share > 0.0)) return 0.0;
    // Gray levels, weighted as cvtColor weights them, since that is the
    // picture segmentObject holds them against.
    const cv::Mat cur = strideSample(curBgr, 4);
    const cv::Mat bg = strideSample(bgBgr, 4);
    const cv::Mat on = strideSample(mask, 4);
    std::vector<double> cloth, base;
    for (int y = 0; y < cur.rows; ++y) {
        const unsigned char* c = cur.ptr<unsigned char>(y);
        const unsigned char* b = bg.ptr<unsigned char>(y);
        const unsigned char* o = on.ptr<unsigned char>(y);
        for (int x = 0; x < cur.cols; ++x) {
            if (!o[x]) continue;
            const int i = x * 3;
            cloth.push_back(0.114 * c[i] + 0.587 * c[i + 1] + 0.299 * c[i + 2]);
            base.push_back((0.114 * b[i] + 0.587 * b[i + 1] + 0.299 * b[i + 2]) * gain);
        }
    }
    if (cloth.empty()) return 0.0;
    const double lay = median(std::move(cloth));
    const double under = median(std::move(base));
    if (!(under > 0.0) || lay > cfg.dark_garment_black_share * under) return 0.0;
    return std::min(cfg.dark_garment_black_margin * lay, cfg.dark_garment_black_cap * under);
}

bool largestContour(const cv::Mat& mask, double minAreaPx, std::vector<cv::Point>& contour) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.empty()) return false;

    std::size_t best = 0;
    double bestArea = cv::contourArea(contours[0]);
    for (std::size_t k = 1; k < contours.size(); ++k) {
        const double a = cv::contourArea(contours[k]);
        if (a > bestArea) {
            bestArea = a;
            best = k;
        }
    }
    if (bestArea < minAreaPx) return false;

    contour = contours[best];
    return true;
}

}  // namespace verimeasure
