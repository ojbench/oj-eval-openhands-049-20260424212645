#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    /////////////////////////////////
    /// TODO: You can add any [private] member variable or [private] member function you need.
    /////////////////////////////////

public:

    Vec get_v_next() {
        int n = monitor->get_robot_number();
        Vec pref_v = (pos_tar - pos_cur);
        double dist = pref_v.norm();
        
        if (dist < EPSILON) return Vec(0, 0);

        // Scale pref_v to reach target in one step if possible, otherwise v_max
        if (dist > v_max * TIME_INTERVAL) {
            pref_v = pref_v * (v_max / dist);
        } else {
            pref_v = pref_v * (1.0 / TIME_INTERVAL);
        }

        const double T_hor = 3.0;
        const double buffer = 0.05;
        const double v_max_sqr = v_max * v_max;

        struct OtherRobot {
            Vec p, v;
            double r_sum_sqr;
        };
        std::vector<OtherRobot> others;
        others.reserve(n);
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            others.push_back({monitor->get_pos_cur(j), monitor->get_v_cur(j), 
                              pow(r + monitor->get_r(j) + buffer, 2)});
        }

        auto get_score = [&](Vec v) {
            double v_norm_sqr = v.norm_sqr();
            if (v_norm_sqr > v_max_sqr + 1e-7) return -2e18;
            
            double min_t_collision = T_hor;
            bool collision_imminent = false;
            double penalty = 0;

            for (const auto& other : others) {
                Vec dp = pos_cur - other.p;
                Vec dv = v - other.v;
                
                double a = dv.norm_sqr();
                double b = 2 * dp.dot(dv);
                double c = dp.norm_sqr() - other.r_sum_sqr;
                
                if (c < 0) {
                    if (b < 0) return -1e18 + b; 
                    else continue; 
                }
                
                if (b < 0 && a > 1e-9) {
                    double disc = b * b - 4 * a * c;
                    if (disc > 0) {
                        double t = (-b - sqrt(disc)) / (2 * a);
                        if (t < T_hor) {
                            if (t < min_t_collision) min_t_collision = t;
                            collision_imminent = true;
                        }
                    }
                }
                
                if (dp.dot(dv) < 0) {
                    double cp = dp.cross(dv);
                    penalty += 0.02 * std::max(0.0, -cp); 
                }
            }

            if (collision_imminent) {
                return -1e15 + min_t_collision * 1e10;
            } else {
                return -(v - pref_v).norm() - penalty;
            }
        };

        Vec best_v = Vec(0, 0);
        double best_score = get_score(best_v);

        auto update_best = [&](Vec v) {
            double s = get_score(v);
            if (s > best_score) {
                best_score = s;
                best_v = v;
            }
        };

        update_best(pref_v);
        update_best(v_cur);
        
        const int num_dirs = 24;
        const int num_speeds = 2;
        for (int i = 0; i < num_dirs; ++i) {
            double theta = 2.0 * M_PI * i / num_dirs;
            Vec dir(cos(theta), sin(theta));
            for (int s = 1; s <= num_speeds; ++s) {
                update_best(dir * (v_max * s / num_speeds));
            }
        }
        
        for (int i = 1; i <= 4; ++i) {
            double angle = (i - 2.5) * 0.2;
            update_best(pref_v.rotate(angle));
            update_best(v_cur.rotate(angle));
        }

        return best_v;
    }
};


/////////////////////////////////
/// TODO: You can add any class or struct you need.
/////////////////////////////////


#endif //PPCA_SRC_HPP