import { reactive } from "vue";

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

// 数据类型选项
export const dataTypeOptions = reactive<{ value: string; label: string }[]>([]);

// 度量类型选项
export const metricTypeOptions = [
  { value: 1, label: '当前数值' },
  { value: 2, label: '累计增量' },
  { value: 3, label: '累计减量' },
  { value: 4, label: '历史存在' },
  { value: 5, label: '位于区域' },
  { value: 6, label: '临近航点' },
];

// 时间范围类型选项
export const temporalScopeOptions = [
  { value: 1, label: '实时' },
  { value: 2, label: '全周期' },
  { value: 3, label: '滚动窗口' }
];


export const initCondition = () => {
  fetch('/rules/condition_rules.json')
    .then(response => response.json())
    .then(data => {
      if (Array.isArray(data.rules)) {
        data.rules.forEach((item: { type: string; description: string }) => {
          dataTypeOptions.push({ value: item.type, label: item.description });
        });
      }
      console.log('成功加载数据类型:', dataTypeOptions.length);
    })
    .catch(error => {
      console.error('加载文件时出错:', error);
    });
};

initCondition();
// 统计类型维度 metricType
// 1: 当前数值 顾名思义，判断最新的数值

// 2: 累计增量 统计时间维度设置中数值的累计增加量，比如：统计比赛开始后一共兑换多少发弹量

// 3: 累计减量 统计时间维度设置中数值的累计增加量，比如：统计比赛开始后一共消耗多少发弹量

// 4: 历史存在 统计时间维度设置中数值是否出现过，比如: 是否最近 10s 内是否检测到敌方机器人

// 时间范围维度 temporalScope
// 1: 全周期 统计比赛开始后的所有数据

// 2: 任务周期 统计执行任务开始至今的所有数据

// 3: 滚动周期 统计最近一段时间范围内的所有数据