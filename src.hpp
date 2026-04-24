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

        if (dist > v_max) {
            pref_v = pref_v.normalize() * v_max;
        } else {
            // If close to target, we might want to slow down to exactly reach it
            // but the simulator allows moving at v_max as long as we don't overshoot.
            // Actually, the simulator says we don't stop automatically.
        }

        const double T_hor = 4.0;
        const double buffer = 0.1;

        auto get_score = [&](Vec v) {
            if (v.norm() > v_max + 1e-7) return -2e18;
            
            double min_t_collision = T_hor;
            bool collision_imminent = false;
            double penalty = 0;

            for (int j = 0; j < n; ++j) {
                if (j == id) continue;
                Vec pj = monitor->get_pos_cur(j);
                Vec vj = monitor->get_v_cur(j);
                double rj = monitor->get_r(j);
                
                Vec dp = pos_cur - pj;
                Vec dv = v - vj;
                double R = r + rj + buffer;
                
                double a = dv.norm_sqr();
                double b = 2 * dp.dot(dv);
                double c = dp.norm_sqr() - R * R;
                
                if (c < 0) {
                    if (b < 0) return -1e18 + b; // Getting worse, prioritize moving apart
                    else continue; 
                }
                
                if (b < 0) {
                    // Check if collision occurs within T_hor
                    // Quadratic: a*t^2 + b*t + c = 0
                    double disc = b * b - 4 * a * c;
                    if (disc > 0) {
                        double t = (-b - sqrt(disc)) / (2 * a);
                        if (t < T_hor) {
                            if (t < min_t_collision) min_t_collision = t;
                            collision_imminent = true;
                        }
                    }
                }
                
                // Right-hand bias: prefer velocities that pass to the right
                // Relative position dp, relative velocity dv.
                // We want dv to have a positive cross product with dp? 
                // No, if dp is from j to i, and dv is i's velocity relative to j.
                // If dv is pointing towards j, we want it to steer right.
                double cp = dp.cross(dv);
                penalty += 0.01 * std::max(0.0, -cp); // Small penalty for passing on the left
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
            if (v.norm() > v_max) v = v.normalize() * v_max;
            double s = get_score(v);
            if (s > best_score) {
                best_score = s;
                best_v = v;
            }
        };

        update_best(pref_v);
        update_best(v_cur);
        update_best(Vec(0, 0));
        
        const int num_dirs = 72;
        const int num_speeds = 4;
        for (int i = 0; i < num_dirs; ++i) {
            double theta = 2.0 * M_PI * i / num_dirs;
            Vec dir(cos(theta), sin(theta));
            for (int s = 1; s <= num_speeds; ++s) {
                update_best(dir * (v_max * s / num_speeds));
            }
        }
        
        // Add some samples around pref_v and v_cur
        for (int i = 1; i <= 10; ++i) {
            double angle = (i - 5.5) * 0.2;
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