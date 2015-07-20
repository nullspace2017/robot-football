#include "vision.h"
#include <iostream>
#include <cstdio>
#include <cassert>
#include <functional>
using namespace std;
using namespace cv;

cv::Vec3b const Vision::const_vcolors[] = {
    Vec3b(255, 255, 255),
    Vec3b(0, 180, 0),
    Vec3b(0, 0, 0),
    Vec3b(0, 0, 255),
    Vec3b(255, 0, 0),
    Vec3b(31, 31, 31)
};

Vision::Vision(int height, int width, Transform *trans):
    height(height), width(width), trans(trans) {
    v_pic = new uchar *[height];
    v_pic_pool = new uchar[height * width];
    for (int i = 0; i < height; i++)
        v_pic[i] = v_pic_pool + i * width;
    v_plat = new uchar *[VPLAT_HEIGHT];
    v_plat_pool = new uchar[VPLAT_HEIGHT * VPLAT_WIDTH];
    for (int i = 0; i < VPLAT_HEIGHT; i++)
        v_plat[i] = v_plat_pool + i * VPLAT_WIDTH;
}

Vision::~Vision() {
    delete[] v_pic;
    delete[] v_pic_pool;
    delete[] v_plat;
    delete[] v_plat_pool;
}

void Vision::input(Mat const &in) {
    assert(in.rows == height);
    assert(in.cols == width);
    assert(in.dims == 2);
    pic = in.clone();
    pre_copy();
    get_edge_white();
    update_plat();
}

cv::Mat Vision::gen_as_pic() {
    Mat out(height, width, CV_8UC3);
    for (int i = 0; i < out.rows; i++) {
        uchar *puchar = v_pic[i];
        Vec3b *pmat = out.ptr<Vec3b>(i);
        for (int j = 0; j < out.cols; j++) {
            pmat[j] = const_vcolors[puchar[j]];
        }
    }
    return out;
}

cv::Mat Vision::gen_platform() {
    Mat out(VPLAT_HEIGHT, VPLAT_WIDTH, CV_8UC3);
    for (int i = 0; i < out.rows; i++) {
        uchar *puchar = v_plat[i];
        Vec3b *pmat = out.ptr<Vec3b>(i);
        for (int j = 0; j < out.cols; j++) {
            pmat[j] = const_vcolors[puchar[j]];
        }
    }
    return out;
}

void Vision::pre_copy() {
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
    for (int i = 0; i < pic.rows; i++) {
        uchar *puchar = v_pic[i];
        Vec3b const *pmat = pic.ptr<Vec3b const>(i);
        for (int j = 0; j < pic.cols; j++) {
            Vec3b const &pixel = pmat[j];
            uchar c = get_color(pixel[2], pixel[1], pixel[0]);
            puchar[j] = c;
        }
    }
}

void Vision::get_edge_white() {
    // 这里的坐标系: xy与ij方向一致,即x=v,y=u
    for (int i = 0; i < height; i += 2) {
        for (int j = 0; j < width; j += 2) {
            if (v_pic[i][j] != VCOLOR_WHITE) continue;
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
                    add_to_cnt(v_pic[k][y1]);
                    add_to_cnt(v_pic[k][y2]);
                }
                for (int k = y1 + 1; k < y2; k++) {
                    add_to_cnt(v_pic[x1][k]);
                    add_to_cnt(v_pic[x2][k]);
                }
                int cnt_total = cnt_wh + cnt_gr + cnt_bk;
                if (cnt_bk < cnt_total * 0.2 && cnt_wh > cnt_total * 0.1 && cnt_gr > cnt_total * gr_rate[m]) {
                    if (special_check) {
                        cnt_wh = 0;
                        int x;
                        for (x = i; x >= 0; x--) {
                            if (v_pic[x][j] == 0) cnt_wh++;
                            else break;
                        }
                        int expect_gr = cnt_wh * 0.8;
                        if (expect_gr < 10) expect_gr = 10;
                        cnt_gr = 0;
                        int startp = x;
                        for (; x >= 0 && x < startp + cnt_gr; x--) {
                            if (v_pic[x][j] == 1) cnt_gr++;
                        }
                        if (cnt_gr > expect_gr * 0.5) {
                            v_pic[i][j] = VCOLOR_EDGE_POSSIBLE;
                            break;
                        }
                    } else {
                        v_pic[i][j] = VCOLOR_EDGE_POSSIBLE;
                        break;
                    }
                }
            }
            if (v_pic[i][j] == VCOLOR_EDGE_POSSIBLE) {
                int cnt_poss = 0;
                int x1 = i - 4;
                int x2 = i + 4;
                int y1 = j - 4;
                int y2 = j + 4;
                cut_to_rect(x1, y1);
                cut_to_rect(x2, y2);
                for (int x = x1; x < x2; x += 2) {
                    for (int y = y1; y < y2; y += 2) {
                        if (v_pic[x][y] == VCOLOR_EDGE_POSSIBLE)
                            cnt_poss++;
                    }
                }
                if (cnt_poss > 3) {
                    v_pic[i][j] = VCOLOR_EDGE;
                    static function<void(int, int)> expand_to_white = [&](int x, int y) {
                        int vx[] = {x - 1, x + 1, x, x};
                        int vy[] = {y, y, y - 1, y + 1};
                        for (int i = 0; i < 4; i++) {
                            int nx = vx[i];
                            int ny = vy[i];
                            if (in_rect(nx, ny) && (v_pic[nx][ny] == VCOLOR_WHITE || v_pic[nx][ny] == VCOLOR_EDGE_POSSIBLE)) {
                                v_pic[nx][ny] = VCOLOR_EDGE;
                                expand_to_white(nx, ny);
                            }
                        }
                    };
                    expand_to_white(i, j);
                }
            }
        }
    }
}

void Vision::update_plat() {
    for (int i = 0; i < VPLAT_HEIGHT; i++) {
        for (int j = 0; j < VPLAT_WIDTH; j++) {
            double x = (double)(j - VPLAT_WIDTH / 2) * VPLAT_MM_PER_PIXEL;
            double y = (double)(VPLAT_HEIGHT - 1 - i) * VPLAT_MM_PER_PIXEL;
            Vec2d uv = trans->xy_to_uv(x, y);
            int u = (int)uv[0], v = (int)uv[1];
            if (in_rect(v, u))
                v_plat[i][j] = v_pic[v][u];
            else
                v_plat[i][j] = VCOLOR_OUT_OF_RANGE;
        }
    }
}

void Vision::get_white_lines() {
    Mat white_region(VPLAT_HEIGHT, VPLAT_WIDTH, CV_8U, Scalar::all(0));
    for (int i = 0; i < white_region.rows; i ++) {
        uchar *pwhite = white_region.ptr<uchar>(i);
        for (int j = 0; j < white_region.cols; j ++) {
            if (v_plat[i][j] == VCOLOR_EDGE)
                pwhite[j] = 255;
        }
    }
    Canny(white_region, white_region, 200, 50);
    vector<Vec2f> lines;
    HoughLines(white_region, lines, 1, CV_PI/180, 20);
    Mat angles(lines.size(), 1, CV_32F), labels(lines.size(), 1, CV_32S);
    cout << lines.size() << endl;
    for( size_t i = 0; i < lines.size(); i++ ) {
        angles.at<float>(i, 1) = lines[i][0];
        //cout << angles.at<float>(i, 1) << endl;
//        float rho = lines[i][0];
//        float theta = lines[i][1];
//        double a = cos(theta), b = sin(theta);
//        double x0 = a*rho, y0 = b*rho;
//        Point pt1(cvRound(x0 + 1000*(-b)),
//                  cvRound(y0 + 1000*(a)));
//        Point pt2(cvRound(x0 - 1000*(-b)),
//                  cvRound(y0 - 1000*(a)));
//        line(white_region, pt1, pt2, Scalar(128,0,255), 1, 8 );
    }
    kmeans(angles, 2, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.1), 3, KMEANS_PP_CENTERS);
    for (size_t i = 0; i < lines.size(); i ++) {
        if (labels.at<int>(i, 1) != 0) continue;
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        line(white_region, pt1, pt2, Scalar(128,0,255), 1, 8 );
    }
    imshow("white_region", white_region);
    waitKey();
}
