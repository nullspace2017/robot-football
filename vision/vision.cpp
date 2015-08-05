#include "vision.h"
#include <iostream>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <cassert>
#include <algorithm>
#include <functional>
using namespace std;
using namespace cv;

cv::Vec3b const Vision::const_vcolors[] = {
    Vec3b(255, 255, 255),
    Vec3b(0, 180, 0),
    Vec3b(0, 0, 0),
    Vec3b(0, 0, 255),
    Vec3b(255, 0, 0),
    Vec3b(31, 31, 31),
    Vec3b(0,0,127), //huanglj
    Vec3b(127,0,0) //huanglj
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
    get_white_lines();
    match_robot_pos();
    get_ball_hough();
}

void Vision::get_location(Vec2f &pos, Vec2f &direct, double &location_confidence) {
    pos = robot_pos;
    direct = robot_direct;
    location_confidence = robot_location_confidence;
}

void Vision::get_ball_pos(Vec2f &pos, Vision::BALLSTATE &ball_state) {
    pos = trans->uv_to_xy(ballx, bally + ballr);
    ball_state = this->ball_state;
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
    for (size_t i = 0; i < white_lines.size(); i++) {
        double rho = white_lines[i][0], theta = white_lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        line(out, pt1, pt2, Scalar(128,0,255), 1, 8 );
    }
    return out;
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
            double y = (double)(VPLAT_HEIGHT - i) * VPLAT_MM_PER_PIXEL;
            Vec2d uv = trans->xy_to_uv(x, y);
            int u = cvRound(uv[0]), v = cvRound(uv[1]);
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
    vector<Vec2f> lines, filt_lines;
    HoughLines(white_region, lines, 1, CV_PI/180, 25);
    const float dr[3] = {0, 20.0 / VPLAT_MM_PER_PIXEL, -20.0 / VPLAT_MM_PER_PIXEL};
    auto traverse_line = [](float rho, float theta, function<void(int v, int u)> vis) {
        float ct = cos(theta), st = sin(theta);
        if (theta < CV_PI / 4 || theta > CV_PI * 3 / 4) {
            for (int h = 0; h < VPLAT_HEIGHT; h ++) {
                int w = cvRound((rho/st-(float)h)*st/ct);
                if (w < 0 || w >= VPLAT_WIDTH) continue;
                vis(h, w);
            }
        } else {
            for (int w = 0; w < VPLAT_WIDTH; w ++) {
                int h = cvRound((rho/ct-(float)w)*ct/st);
                if (h < 0 || h >= VPLAT_HEIGHT) continue;
                vis(h, w);
            }
        }
    };
    auto continious_white_point_in_line = [&](float rho, float theta) {
        int max_continious = 0;
        int continious = 0;
        int intervered = 0;
        traverse_line(rho, theta, [&](int v, int u) {
           if (v_plat[v][u] == VCOLOR_EDGE) {
               continious++;
               intervered = 0;
           } else {
               intervered++;
               if (intervered > 2) {
                   if (continious > max_continious)
                       max_continious = continious;
                   continious = 0;
               }
           }
           if (continious > max_continious)
               max_continious = continious;
        });
        return max_continious;
    };
    for (size_t i = 0; i < lines.size(); i ++) {
        for (int k = 0; k < 3; k ++) {
            float rho = lines[i][0] += dr[k], theta = lines[i][1];
            if (continious_white_point_in_line(rho, theta) > 50) {
                filt_lines.push_back(Vec2f(rho, theta));
            }
        }
    }
    lines.clear();
    int part_lines_count[6];
    memset(part_lines_count, 0, sizeof(part_lines_count));
    random_shuffle(filt_lines.begin(), filt_lines.end());
    for (size_t i = 0; i < filt_lines.size(); i++) {
        int ind = (int)(filt_lines[i][1] * 6.0f / CV_PI);
        if (part_lines_count[ind] < 10) {
            part_lines_count[ind]++;
            lines.push_back(filt_lines[i]);
        }
    }
    vector<vector<Point> > point_set(lines.size());
    vector<int> classify(lines.size(), 1);
    int max_class_num = 0;
    for (size_t i = 0; i < lines.size(); i++) {
        if (classify[i] == 0) continue;
        vector<Point> &s = point_set[i];
        traverse_line(lines[i][0], lines[i][1], [&](int v, int u) {
            if (v_plat[v][u] == VCOLOR_EDGE) s.push_back(Point(v, u));
        });
    }
    auto line_included = [&](int line_1_index, int line_2_index) {
        if (line_1_index == line_2_index) return true;
        vector<Point> const &ps1(point_set[line_1_index]), &ps2(point_set[line_2_index]);
        if (ps1.size() == 0) return true;
        int test_times = 100;
        int similar = 0;
        for (int i = 0; i < test_times; i++) {
            Point p1 = ps1[rand() % ps1.size()];
            for (size_t j = 0; j < ps2.size(); j++) {
                Point delta = ps2[j] - p1;
                int dist_2 = delta.dot(delta);
                if (dist_2 < 70 * 70 / (VPLAT_MM_PER_PIXEL * VPLAT_MM_PER_PIXEL)) {
                    similar++;
                    break;
                }
            }
        }
        if (similar > test_times * 0.6) return true;
        else return false;
    };
    int last_class_num = -1;
    while (last_class_num != max_class_num) {
        last_class_num = max_class_num;
        max_class_num = 0;
        for (size_t i = 0; i < classify.size(); i++) {
            if (classify[i] <= 0) classify[i] = 0;
        }
        for (size_t i = 0; i < lines.size(); i++) {
            if (classify[i] == 0) continue;
            size_t j;
            for (j = 0; j < i; j++) {
                if (classify[j] <= 0) continue;
                if (point_set[i].size() <= point_set[j].size()) {
                    if (line_included(i, j)) {
                        classify[i] = -classify[j];
                        break;
                    } else if (line_included(j, i)){
                        classify[i] = classify[j];
                        classify[j] = -classify[i];
                        break;
                    }
                } else {
                    if (line_included(j, i)){
                        classify[i] = classify[j];
                        classify[j] = -classify[i];
                        break;
                    } else if (line_included(i, j)) {
                        classify[i] = -classify[j];
                        break;
                    }
                }
            }
            if (j == i) {
                classify[i] = ++max_class_num;
            }
        }
    }
    white_lines.clear();
    for (size_t i = 0; i < lines.size(); i++) {
        if (classify[i] > 0)
            white_lines.push_back(lines[i]);
    }
}

void Vision::match_robot_pos() {
    if (white_lines.size() == 0) {
        robot_location_confidence = 0;
        return;
    }
    static float const add_angle_per_times = CV_PI / 180;
    float min_error = 1e20;
    float delta_base = 0;
    for (float angle = 0; angle < CV_PI / 2; angle += add_angle_per_times) {
        float error = 0;
        for (size_t i = 0; i < white_lines.size(); i++) {
            float theta = white_lines[i][1] + angle;
            while (theta > CV_PI / 4) theta -= CV_PI / 2;
            while (theta < -CV_PI / 4) theta += CV_PI / 2;
            error += theta * theta;
        }
        if (error < min_error) {
            min_error = error;
            delta_base = angle;
        }
    }
    vector<Point2f> white_point_in_robot[4];
    for (int m = 0; m < 4; m++) {
        float delta = delta_base + CV_PI / 2 * m;
        Vec2d i_robot_to_world(cos(delta), -sin(delta)), j_robot_to_world(sin(delta), cos(delta));
        for (int i = 0; i < VPLAT_HEIGHT; i += 4) {
            for (int j = 0; j < VPLAT_WIDTH; j += 4) {
                if (v_plat[i][j] == VCOLOR_EDGE) {
                    float x = (float)(j - VPLAT_WIDTH / 2) * VPLAT_MM_PER_PIXEL;
                    float y = (float)(VPLAT_HEIGHT - i) * VPLAT_MM_PER_PIXEL;
                    float xw = x * i_robot_to_world[0] + y * j_robot_to_world[0];
                    float yw = x * i_robot_to_world[1] + y * j_robot_to_world[1];
                    white_point_in_robot[m].push_back(Point2f(xw, yw));
                }
            }
        }
    }
    vector<Point2f> ground_line_normal;
    for (size_t i = 0; i < ground.lines.size(); i++) {
        Vec4f line = ground.lines[i];
        Vec2f n(line[3] - line[1], line[0] - line[2]);
        ground_line_normal.push_back(n / sqrt(n.dot(n)));
    }
    static auto nearest_line_dist_2 = [&](Point2f const &p) {
        float min_dist = 1e20;
        for (size_t i = 0; i < ground.lines.size(); i++) {
            Vec4f const &line(ground.lines[i]);
            Point2f p1(line[0], line[1]), p2(line[2], line[3]);
            float dist;
            Point2f vec_p_p1 = p - p1;
            Point2f vec_p_p2 = p - p2;
            Point2f vec_p2_p1 = p2 - p1;
            if (vec_p2_p1.dot(vec_p_p1) <= 0) dist = vec_p_p1.dot(vec_p_p1);
            else if (vec_p2_p1.dot(vec_p_p2) >= 0) dist = vec_p_p2.dot(vec_p_p2);
            else { dist = ground_line_normal[i].dot(vec_p_p1); dist *= dist; }
            if (dist < min_dist)
                min_dist = dist;
        }
        return min_dist;
    };
    vector<pair<float, pair<int, Point2f> > > verr;
    min_error = 1e20;
    if (white_lines.size() < 10) {
        Mat view_ground = ground.gen_ground_view();
        for (int m = 0; m < 4; m++) {
            //float delta = delta_base + CV_PI / 2 * m;
            vector<Point2f> possible_pos;
            for (int i = 0; i <= 1800; i += 50) {
                for (int j = 0; j <= 2200; j += 50) {
                    line(view_ground, ground.xy_to_uv(i, j), ground.xy_to_uv(i, j), Scalar::all(127), 3);
                    possible_pos.push_back(Point2f(i, j));
                }
            }
            for (size_t i = 0; i < possible_pos.size(); i++) {
                Point2f supposed_pos(possible_pos[i]);
                float sum_error = 0;
                float min_filt = min_error * white_point_in_robot[m].size();
                for (auto it = white_point_in_robot[m].begin(); it != white_point_in_robot[m].end(); it++) {
                    float err = nearest_line_dist_2(*it + supposed_pos);
                    sum_error += err;
                    if (sum_error > min_filt * 3)
                        break;
                }
                sum_error /= white_point_in_robot[m].size();
                if (sum_error < min_error)
                    min_error = sum_error;
                verr.push_back(make_pair(sum_error, make_pair(m, supposed_pos)));
            }
        }
        sort(verr.begin(), verr.end(), [](pair<float, pair<int, Point2f> > const &a, pair<float, pair<int, Point2f> > const &b) {
            return a.first < b.first; });
        pair<int, Point2f> min_loc = verr.front().second;
        bool result_unique = true;
        for (size_t i = 1; i < verr.size() && verr[i].first < min_error * 3; i++) {
            pair<int, Point2f> const &loc = verr[i].second;
            Point2f s = min_loc.second - loc.second;
            if (loc.first != min_loc.first || s.dot(s) > 300 * 300) {
                result_unique = false;
                break;
            }
        }
        robot_location_confidence = 0.8;
        if (result_unique == false)
            robot_location_confidence = 0;
        if (white_point_in_robot[min_loc.first].size() < 100)
            robot_location_confidence = 0;
        if (min_error > 2000)
            robot_location_confidence = 0;
        robot_pos = min_loc.second;
        float robot_delta = delta_base + CV_PI / 2 * min_loc.first;
        robot_direct = Vec2f(sin(robot_delta), cos(robot_delta));
        if (robot_location_confidence > 0.5) {
            int m = min_loc.first;
            Point2f min_pos(robot_pos[0], robot_pos[1]);
            float min_delta = robot_delta;
            ground.draw_robot(view_ground, min_pos, Vec2f(sin(min_delta), cos(min_delta)));
            for (auto it = white_point_in_robot[m].begin(); it != white_point_in_robot[m].end(); it++) {
                Point2f p = *it + min_pos;
                line(view_ground, ground.xy_to_uv(p.x, p.y), ground.xy_to_uv(p.x, p.y), Scalar(0, 0, 191), 2);
            }
        }
        imshow("view_ground", view_ground);
    } else {
        robot_location_confidence = 0;
    }
}

void Vision::get_ball_color() { //huanglj
    int iLowH = 155, iHighH = 179, iLowS = 40, iHighS = 255, iLowV = 60, iHighV = 255;

    Mat imgHSV;
    cvtColor(pic, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    for (int i = 0; i < height; i ++) {
        uchar *dt = imgThresholded.ptr<uchar>(i);
//        uchar *di = pic.ptr<uchar>(i);
        for (int j = 0; j < width; j ++) {
//            uchar b = di[j*pic.channels()], g = di[j*pic.channels()+1], r = di[j*pic.channels()+2];
            if (v_pic[i][j] == VCOLOR_WHITE || v_pic[i][j] == VCOLOR_EDGE || v_pic[i][j] == VCOLOR_GREEN) {
                dt[j] = 0;
                continue;
            }
        }
    }


    //morphological closing (removes small holes from the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );

    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)) );

    for (int i = 0; i < height; i ++) {
        uchar *dt = imgThresholded.ptr<uchar>(i);
        for (int j = 0; j < width; j ++) {
            if (dt[j] == 255) v_pic[i][j] = VCOLOR_BALL_POSSIBLE;
        }
    }

    for (int i = 0; i < height; i += 2) {
        for (int j = 0; j < width; j += 2) {
            if (v_pic[i][j] != VCOLOR_BALL_POSSIBLE) continue;
            static int const dx = 20, dy = 20;
            static double const gr_rate = 0.4;
            int x1 = i - dx, x2 = i + dx;
            int y1 = j - dy, y2 = j + dy;
            cut_to_rect(x1, y1);
            cut_to_rect(x2, y2);
            int cnt_gr = 0, cnt_total = 0;
            static auto add_to_cnt = [&](uchar color) {
                cnt_total ++;
                if (color == VCOLOR_GREEN) cnt_gr ++;
            };
            for (int k = x1; k <= x2; k++) {
                add_to_cnt(v_pic[k][y1]);
                add_to_cnt(v_pic[k][y2]);
            }
            for (int k = y1 + 1; k < y2; k++) {
                add_to_cnt(v_pic[x1][k]);
                add_to_cnt(v_pic[x2][k]);
            }

            if (cnt_gr > cnt_total*gr_rate) {
                v_pic[i][j] = VCOLOR_BALL;
                expand_to_ball(i, j);
            }
        }
    }

    int cntx = 0, cnty = 0, cnt_total = 0;
    double cntr = 0;
    for (int i = 0; i < height; i ++) {
        for (int j = 0; j < width; j ++) {
            if (v_pic[i][j] == VCOLOR_BALL) {
                cnt_total ++;
                cntx += j;
                cnty += i;
            }
        }
    }
    if (cnt_total == 0) {
        ball_state = BALL_NO;
    } else {
        ballx = cntx/cnt_total;
        bally = cnty/cnt_total;
        for (int i = 0; i < height; i ++) {
            for (int j = 0; j < width; j ++) {
                if (v_pic[i][j] == VCOLOR_BALL) {
                    cntr += sqrt(double((j - ballx)*(j - ballx) + (i - bally)*(i - bally)));
                }
            }
        }
//        ballr = cntr/cnt_total * 1.5;
        ballr = pow(1.5*cntr/M_PI, 1.0/3);
        cout << ballx << ' ' << bally << ' ' << ballr << '\n';
        cout << cntr << '\n';

        //print
        int x1 = ballx - ballr, x2 = ballx + ballr;
        int y1 = bally - ballr, y2 = bally + ballr;
        cout << x1 << ' ' << y1 << '\n';
        cout << x2 << ' ' << y2 << '\n';
        cut_to_rect(x1, y1);
        cut_to_rect(x2, y2);
        for (int k = x1; k <= x2; k++) {
            v_pic[y1][k] = VCOLOR_BALL_POSSIBLE;
            v_pic[y2][k] = VCOLOR_BALL_POSSIBLE;
        }
        for (int k = y1 + 1; k < y2; k++) {
            v_pic[k][x1] = VCOLOR_BALL_POSSIBLE;
            v_pic[k][x2] = VCOLOR_BALL_POSSIBLE;
        }

        //判断球是否在机器前
        ball_state = BALL_HAS;
    }
}

void Vision::get_ball_hough() { //huanglj
    // 这里的坐标系: x=u,y=v
    int iLowH = 160, iHighH = 179, iLowS = 60, iHighS = 255, iLowV = 0, iHighV = 255;

    Mat imgHSV;
    cvtColor(pic, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    for (int i = 0; i < height; i ++) {
        uchar *dt = imgThresholded.ptr<uchar>(i);
        for (int j = 0; j < width; j ++) {
            if (v_pic[i][j] == VCOLOR_WHITE || v_pic[i][j] == VCOLOR_EDGE || v_pic[i][j] == VCOLOR_GREEN) {
                dt[j] = 0;
                continue;
            }
        }
    }
    Mat img = pic, gray, can;
    cvtColor(img, gray, CV_BGR2GRAY);
    // smooth it, otherwise a lot of false circles may be detected
    GaussianBlur(gray, gray, Size(9, 9), 2, 2 );
    vector<Vec3f> circles;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1.5, 20, 150, 45);

    int isBall = -1, alterBall = -1;
    double max_rate = 0, max_radius = 0;
    for (size_t i = 0; i < circles.size(); i ++) {
        int centerx = cvRound(circles[i][0]), centery = cvRound(circles[i][1]), centerr = cvRound(circles[i][2]);
        int x1 = centerx - centerr, x2 = centerx + centerr;
        int y1 = centery - centerr, y2 = centery + centerr;
        cut_to_rect(y1, x1);
        cut_to_rect(y2, x2);
        int cnt_total = 0, cnt_ball = 0;
        double ball_rate = 0.3, thresh_rate = 0.8;
        for (int y = y1; y <y2; y ++) {
            double tempc = centerr, tempa = fabs(centery - y);
            double tempb = sqrt(tempc*tempc - tempa*tempa);
            x1 = centerx - (int)tempb;
            x2 = centerx + (int)tempb;
            cut_to_rect(y1, x1);
            cut_to_rect(y2, x2);
            uchar *dt = imgThresholded.ptr<uchar>(y);
            for (int x = x1; x < x2; x ++) {
                cnt_total ++;
                if (dt[x] == 255) cnt_ball ++;
            }
        }
        double temp_rate = cnt_ball*1.0/cnt_total;
        if (temp_rate > ball_rate && temp_rate > thresh_rate && centerr > max_radius) {
            isBall = i;
            max_radius = centerr;
        }
        if (temp_rate > ball_rate && centerr > 10 && temp_rate > max_rate) {
            alterBall = i;
            max_rate = temp_rate;
        }
    }
    if (isBall == -1) isBall = alterBall;

    for (size_t i = 0; i < circles.size(); i++ ) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(img, center, 3, Scalar(0,255,0), -1, 8, 0 );
        if (isBall == (int)i) {
            ballx = circles[i][0];
            bally = circles[i][1];
            ballr = circles[i][2];
            circle(img, center, radius, Scalar(255,0,0), 3, 8, 0);
        } else {
            circle(img, center, radius, Scalar(0,0,255), 1, 8, 0 );
        }
    }
    imshow("thresh", imgThresholded);
    imshow("circle", img);
    if (isBall == -1) {
        ball_state = BALL_NO;
    } else {
        ball_state = BALL_HAS;
    }
}

void Vision::pre_copy() {
    static auto get_color = [&](uchar h, uchar s, uchar v) {
        if (h > 102 && h < 147 && s > 90 && s < 170) return VCOLOR_GREEN;
        if (s < 70 && v > 175) return VCOLOR_WHITE;
        return VCOLOR_BACKGROUND;
    };
    Mat hsv;
    cvtColor(pic, hsv, CV_BGR2HSV_FULL);
    for (int i = 0; i < hsv.rows; i++) {
        uchar *puchar = v_pic[i];
        Vec3b const *pmat = hsv.ptr<Vec3b const>(i);
        for (int j = 0; j < hsv.cols; j++) {
            Vec3b const &pixel = pmat[j];
            uchar c = get_color(pixel[0], pixel[1], pixel[2]);
            puchar[j] = c;
        }
    }
}

void Vision::expand_to_ball(int x, int y) { //huanglj
    int vx[] = {x - 1, x + 1, x, x};
    int vy[] = {y, y, y - 1, y + 1};
    for (int i = 0; i < 4; i++) {
        int nx = vx[i];
        int ny = vy[i];
        if (in_rect(nx, ny) && v_pic[nx][ny] == VCOLOR_BALL_POSSIBLE) {
            v_pic[nx][ny] = VCOLOR_BALL;
            expand_to_ball(nx, ny);
        }
    }
}
