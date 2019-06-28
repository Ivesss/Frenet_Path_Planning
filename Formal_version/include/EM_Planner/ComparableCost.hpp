//
// Created by jydragon on 18-7-21.
//

#include <cmath>

#include <array>

#ifndef LATTICEPLAN_COMPARABLECOST_H
#define LATTICEPLAN_COMPARABLECOST_H

#endif //LATTICEPLAN_COMPARABLECOST_H
class ComparableCost {
public:
    ComparableCost() = default;
    ComparableCost(const bool has_collision, const bool out_of_boundary,
                   const bool out_of_lane, const float safety_cost_,
                   const float smoothness_cost_ , const float historical_cost_)
            : safety_cost(safety_cost_), smoothness_cost(smoothness_cost_)
    ,historical_cost(historical_cost_){
        cost_items[HAS_COLLISION] = has_collision;
        cost_items[OUT_OF_BOUNDARY] = out_of_boundary;
        cost_items[OUT_OF_LANE] = out_of_lane;
    }
    ComparableCost(const ComparableCost &) = default;
    ~ComparableCost()= default;

    int CompareTo(const ComparableCost &other) const {
        for (int i = 0; i < cost_items.size(); ++i) {
            if (cost_items[i]) {
                if (other.cost_items[i]) {
                    continue;
                } else {
                    return 1;//认为该元素cost更大 不更新
                }
            } else {
                if (other.cost_items[i]) {
                    return -1;
                } else {
                    continue;
                }
            }
        }

        constexpr float kEpsilon = 1e-12;
        const float diff = safety_cost + smoothness_cost + historical_cost - other.safety_cost -
                           other.smoothness_cost - other.historical_cost;
        if (std::fabs(diff) < kEpsilon) {
            return 0;
        } else if (diff > 0) {
            return 1;
        } else {
            return -1;
        }
    }
    ComparableCost operator+(const ComparableCost &other) {
        ComparableCost lhs = *this;
        lhs += other;
        return lhs;
    }
    ComparableCost &operator+=(const ComparableCost &other) {
        for (size_t i = 0; i < cost_items.size(); ++i) {
            cost_items[i] = (cost_items[i] || other.cost_items[i]);
        }
        safety_cost += other.safety_cost;
        smoothness_cost += other.smoothness_cost;
        historical_cost += other.historical_cost;
        return *this;
    }
    bool operator>(const ComparableCost &other) const {
        return this->CompareTo(other) > 0;
    }
    bool operator>=(const ComparableCost &other) const {
        return this->CompareTo(other) >= 0;
    }
    bool operator<(const ComparableCost &other) const {
        return this->CompareTo(other) < 0;
    }
    bool operator<=(const ComparableCost &other) const {
        return this->CompareTo(other) <= 0;
    }
    /*
     * cost_items represents an array of factors that affect the cost,
     * The level is from most critical to less critical.
     * It includes:
     * (0) has_collision or out_of_boundary
     * (1) out_of_lane
     *
     * NOTICE: Items could have same critical levels
     */
    //3个参数 是否碰撞 是否出边界 是否出车道
    static const size_t HAS_COLLISION = 0;
    static const size_t OUT_OF_BOUNDARY = 1;
    static const size_t OUT_OF_LANE = 2;
    std::array<bool, 3> cost_items = {{false, false, false}};

    // cost from distance to obstacles or boundaries
    float safety_cost = 0.0f;
    // cost from deviation from lane center, path curvature etc
    float smoothness_cost = 0.0f;
    // cost from distance to previous path
    float historical_cost = 0.0f;
};