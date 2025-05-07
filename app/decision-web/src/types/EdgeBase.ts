import { reactive } from 'vue';
import type { Condition } from '@/types/Condition';
// 边界接口
export interface EdgeId {
  value: number;
}

// 边配置接口
interface EdgeConfig {
  color?: string;
  width?: number;
  dashed?: boolean;
  label?: string;
  type: 'straight' | 'curve';
  curvature?: number;
}

// 响应式Edge类
export class Edge {
  id: EdgeId;
  sourceId: number;
  targetId: number;
  config: EdgeConfig;
  conditions: Condition[];

  constructor(
    id: EdgeId,
    sourceId: number,
    targetId: number,
    config: EdgeConfig | null = null
  ) {
    // 使用reactive创建响应式对象
    this.id = reactive(id);
    this.sourceId = sourceId;
    this.targetId = targetId;

    if (config != null) {
      this.config = reactive(config);
    } else {
      this.config = reactive({
        color: '#94A3B8',
        width: 2,
        dashed: false,
        label: '',
        type: 'curve',
        curvature: 0.5
      });
    }
    this.conditions = reactive([]);
  }

  // 更新样式的方法
  updateStyle(color?: string, width?: number, dashed?: boolean): void {
    if (color) this.config.color = color;
    if (width) this.config.width = width;
    if (dashed !== undefined) this.config.dashed = dashed;
  }

  // 更新连接类型的方法
  updateType(type: 'straight' | 'curve', curvature?: number): void {
    this.config.type = type;
    if (curvature !== undefined && type === 'curve') {
      this.config.curvature = curvature;
    }
  }

  // 更新边的标签
  updateLabel(label: string): void {
    this.config.label = label;
  }

}

export default Edge;
