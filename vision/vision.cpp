#include "vision.h"
#include <iostream>
#include <cstdio>
#include <cassert>
using namespace std;
using namespace cv;

cv::Vec3b const Vision::const_vcolors[] = {
    Vec3b(255, 255, 255),
    Vec3b(0, 180, 0),
    Vec3b(0, 0, 0),
    Vec3b(0, 0, 255),
    Vec3b(255, 0, 0)
};

Vision::Vision(int height, int width, Transform *trans):
    height(height), width(width), trans(trans) {
    v = new uchar *[height];
    v_pool = new uchar[height * width];
    for (int i = 0; i < height; i++)
        v[i] = v_pool + i * width;
}

Vision::~Vision() {
    delete[] v_pool;
    delete[] v;
}

void Vision::input(Mat const &in) {
    assert(in.rows == height);
    assert(in.cols == width);
    assert(in.dims == 2);
    static auto get_color = [&](uchar r, uchar g, uchar b) {
        double H, S, V;
        static auto conv_vsh = [&]{
            int maxx = (unsigned)max(r, max(g, b));
            int minx = (unsigned)min(r, min(g, b));
            V = (double)maxx;
            S = (maxx - minx) / (double)maxx;
            if (r == maxx) H = (r - b) / (double)(maxx - minx) * 60;
            else if (g == maxx) H = 120 + (b - r) / (double)(maxx - minx) * 60;
            else H = 240 + (r - g) / (double)(maxx - minx) * 60;
            if (H < 0) H += 360;
        };
        conv_vsh();
        if (H > 138 && H < 210 && S > 0.3 && V > 80) {
            if (r > 150) return VCOLOR_WHITE;
            return VCOLOR_GREEN;
        } else if (r > 120 && g > 120 && b > 160) {
            return VCOLOR_WHITE;
        }
        return VCOLOR_BACKGROUND;
    };
    pic = in.clone();
    for (int i = 0; i < in.rows; i++) {
        uchar *puchar = v[i];
        Vec3b const *pmat = in.ptr<Vec3b const>(i);
        for (int j = 0; j < in.cols; j++) {
            Vec3b const &pixel = pmat[j];
            uchar c = get_color(pixel[2], pixel[1], pixel[0]);
            puchar[j] = c;
        }
    }
}

void Vision::get_edge_white() {
    for (int i = 0; i < height; i += 2) {
        for (int j = 0; j < width; j += 2) {
            if (v[i][j] != VCOLOR_WHITE) continue;
            static int const dx[] = {6, 10, 9, 22, 60};
            static int const dy[] = {6, 14, 20, 50, 25};
            static double const gr_rate[] = {0.75, 0.75, 0.70, 0.65, 0.65};
            for (int m = 0; m < 5; m++) {
                bool special_check = false;
                int x1 = i - dx[m], x2 = i + dx[m];
                int y1 = j - dy[m], y2 = j + dy[m];
                special_check = cut_to_rect(x1, y1) | cut_to_rect(x2, y2);
                int cnt_wh = 0, cnt_gr = 0, cnt_bk = 0;
                static auto add_to_cnt = [&](uchar color) {
                    if (color == VCOLOR_WHITE) cnt_wh++;
                    else if (color == VCOLOR_BACKGROUND) cnt_bk++;
                    else cnt_gr++;
                };
                for (int k = x1; k <= x2; k++) {
                    add_to_cnt(v[k][y1]);
                    add_to_cnt(v[k][y2]);
                }
                for (int k = y1 + 1; k < y2; k++) {
                    add_to_cnt(v[x1][k]);
                    add_to_cnt(v[x2][k]);
                }
                int cnt_total = cnt_wh + cnt_gr + cnt_bk;
                if (cnt_bk < cnt_total * 0.2 && cnt_wh > cnt_total * 0.1 && cnt_gr > cnt_total * gr_rate[m]) {
                    if (special_check) {
                        cnt_wh = 0;
                        int x;
                        for (x = i; x >= 0; x--) {
                            if (v[x][j] == 0) cnt_wh++;
                            else break;
                        }
                        int expect_gr = cnt_wh * 0.8;
                        if (expect_gr < 10) expect_gr = 10;
                        cnt_gr = 0;
                        int startp = x;
                        for (; x >= 0 && x < startp + cnt_gr; x--) {
                            if (v[x][j] == 1) cnt_gr++;
                        }
                        if (cnt_gr > expect_gr * 0.5) {
                            v[i][j] = VCOLOR_EDGE_POSSIBLE;
                            break;
                        }
                    } else {
                        v[i][j] = VCOLOR_EDGE_POSSIBLE;
                        break;
                    }
                }
            }
            if (v[i][j] == VCOLOR_EDGE_POSSIBLE) {
                int cnt_poss = 0;
                int x1 = i - 4;
                int x2 = i + 4;
                int y1 = j - 4;
                int y2 = j + 4;
                cut_to_rect(x1, y1);
                cut_to_rect(x2, y2);
                for (int x = x1; x < x2; x += 2) {
                    for (int y = y1; y < y2; y += 2) {
                        if (v[x][y] == VCOLOR_EDGE_POSSIBLE)
                            cnt_poss++;
                    }
                }
                if (cnt_poss > 3) {
                    v[i][j] = VCOLOR_EDGE;
                    expand_to_white(i, j);
                }
            }
        }
    }
}

cv::Mat Vision::gen_as_pic() {
    Mat out(height, width, CV_8UC3);
    for (int i = 0; i < out.rows; i++) {
        uchar *puchar = v[i];
        Vec3b *pmat = out.ptr<Vec3b>(i);
        for (int j = 0; j < out.cols; j++) {
            pmat[j] = const_vcolors[puchar[j]];
        }
    }
    return out;
}

cv::Mat Vision::gen_planform() {
    Mat out(700, 600, CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            Vec2d point = trans->uv_to_xy(i, j);
            int x = point[0] / 5 + 300;
            int y = point[1] / 5;
            if (x >= 0 && x < 600 && y >= 0 && y < 700) {
                out.at<Vec3b>(699 - y, x) = pic.at<Vec3b>(i, j);
            }
        }
    }
    return out;
}

void Vision::expand_to_white(int x, int y) {
    int vx[] = {x - 1, x + 1, x, x};
    int vy[] = {y, y, y - 1, y + 1};
    for (int i = 0; i < 4; i++) {
        int nx = vx[i];
        int ny = vy[i];
        if (in_rect(nx, ny) && (v[nx][ny] == VCOLOR_WHITE || v[nx][ny] == VCOLOR_EDGE_POSSIBLE)) {
            v[nx][ny] = VCOLOR_EDGE;
            expand_to_white(nx, ny);
        }
    }
}
