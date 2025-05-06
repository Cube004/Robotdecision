
export interface Condition {
  datetype: string;
  metricType: number;
  temporalScope: {
    type: number;
    rollingWindow: number | null;
  };
  min: number;
  max: number;
}

// 统计类型维度 metricType
// 1: 当前数值 顾名思义，判断最新的数值

// 2: 累计增量 统计时间维度设置中数值的累计增加量，比如：统计比赛开始后一共兑换多少发弹量

// 3: 累计减量 统计时间维度设置中数值的累计增加量，比如：统计比赛开始后一共消耗多少发弹量

// 4: 历史存在 统计时间维度设置中数值是否出现过，比如: 是否最近 10s 内是否检测到敌方机器人

// 时间范围维度 temporalScope
// 1: 全周期 统计比赛开始后的所有数据

// 2: 任务周期 统计执行任务开始至今的所有数据

// 3: 滚动周期 统计最近一段时间范围内的所有数据