import { reactive } from 'vue';
import { type EdgeId } from '@/types/EdgeBase';

export enum NodeType {
  Task = "task", // 任务节点
  Root = "root", // 根节点
  Branch = "branch",// 分支节点
}

export enum Mode {
  Stay = "Stay",
  Move = "Move",
  Limit = "Limit",
}

interface NodeId {
  value: number;
}

// 位置接口
interface Position {
  x: number;
  y: number;
}

// 形状接口
interface Shape {
  width: number;
  height: number;
  borderWidth: number;
  borderRadius: number;
}

// 颜色接口
interface Color {
  borderColor: string;
  fillColor: string;
  fillOpacity: number;
}

// 文本接口
interface Text {
  size: number;
  color: string;
  content: string;
  fontFamily: string;
}

// 图标接口
interface Icon {
  text?: string;
  bgColor?: string;
  src?: string;
  svgPath?: string;
}

// 任务配置接口
interface TaskConfig {
  nodeType: string;
  mode: string;
  waypointId: number | null;
  spin: number | null;
  resetTime: number | null;
  linear: number | null;
}

// 响应式Node类
class Node {
  id: NodeId;
  position: Position;
  shape: Shape;
  color: Color;
  text: Text;
  taskConfig: TaskConfig;
  icon?: Icon;
  edges: EdgeId[];
  constructor(
    id: NodeId,
    position: Position,
    shape: Shape,
    color: Color,
    text: Text | null = null,
    taskConfig: TaskConfig | null = null,
    icon: Icon | null = null,
  )
  {
    // 使用ref创建基本类型的响应式值
    this.id = reactive(id);

    // 使用reactive创建响应式对象
    this.position = reactive({
      x: position.x,
      y: position.y
    });

    this.shape = reactive({
      width: shape.width,
      height: shape.height,
      borderWidth: shape.borderWidth,
      borderRadius: shape.borderRadius
    });

    this.color = reactive({
      borderColor: color.borderColor,
      fillColor: color.fillColor,
      fillOpacity: color.fillOpacity
    });

    if (text != null) {
      this.text = reactive(text);
    } else {
      this.text = reactive({
        size: 12,
        color: "#000000",
        content: `Node ${this.id.value}`, // 注意这里使用.value访问ref值
        fontFamily: 'Arial, sans-serif'
      });
    }

    if (taskConfig != null) {
      this.taskConfig = reactive(taskConfig);
    } else {
      this.taskConfig = reactive({
        nodeType: NodeType.Task,
        mode: Mode.Stay,
        waypointId: null,
        spin: 0,
        linear: 0,
        resetTime: 0,
      });
    }

    if (icon != null) {
      this.icon = reactive(icon);
    }

    this.edges = reactive([]);
  }

  // 更新位置的方法
  updatePosition(x: number, y: number): void {
    this.position.x = x;
    this.position.y = y;
  }

  // 更新颜色的方法
  updateColor(borderColor?: string, fillColor?: string, fillOpacity?: number): void {
    if (borderColor) this.color.borderColor = borderColor;
    if (fillColor) this.color.fillColor = fillColor;
    if (fillOpacity !== undefined) this.color.fillOpacity = fillOpacity;
  }

  // 更新文本的方法
  updateText(content: string, size?: number, color?: string): void {
    this.text.content = content;
    if (size) this.text.size = size;
    if (color) this.text.color = color;
  }

  // 更新图标的方法
  updateIcon(icon: Icon): void {
    this.icon = reactive(icon);
  }
}

export default Node;
