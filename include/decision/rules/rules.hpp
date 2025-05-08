#ifndef RULES_H
#define RULES_H

#include <iostream>
#include <vector>
#include <functional>
#include "roborts_msgs/driver.h"

namespace rules {

    enum metricType{
        CURRENT_VALUE = 1,       // 当前数值
        TOTAL_INCREMENT = 2,     // 累计增量
        TOTAL_DECREMENT = 3,     // 累计减量
        HISTORICAL_PRESENCE = 4, // 历史存在
        IN_REGION = 5,           // 位于区域
        NEAR_WAYPOINT = 6,       // 临近航点
    };

    enum temporalScope{     
        REAL_TIME = 1,       // 实时
        FULL_CYCLE = 2,     // 全周期
        ROLLING_WINDOW = 3, // 滚动周期
    };

    // 规则条件结构体
    struct RuleCondition {
        metricType metric_type;   // 指标类型
        std::string datatype;         // 条件名称
        temporalScope temporal_scope; // 时间范围
        double scope_value;          // 时间范围值
        double min_value;          // 最小值（对于范围判断）或比较值
        double max_value;          // 最大值（对于范围判断）
        RuleCondition(
            int metric_type, 
            std::string datatype, 
            int temporal_scope, 
            double scope_value,
            double max_value,
            double min_value
        ) : 
        min_value(min_value), 
        max_value(max_value), 
        metric_type(metricType(metric_type)), 
        temporal_scope(temporalScope(temporal_scope)), 
        scope_value(scope_value), datatype(datatype) {
        }
        void change_value(double min_value, double max_value, int metric_type, int temporal_scope, double scope_value){
            this->min_value = min_value;
            this->max_value = max_value;
            this->metric_type = (metricType)metric_type;
            this->temporal_scope = (temporalScope)temporal_scope;
            this->scope_value = scope_value;
        }
    };

    // 决策规则结构体
    class DecisionRules {
    public:
        std::vector<RuleCondition> conditions;
        void add_condition(RuleCondition condition){
            conditions.push_back(condition);
        }
    };

}; // namespace rules

#endif // RULES_H