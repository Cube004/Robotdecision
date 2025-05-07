import { type Position } from './Point';
import { reactive } from 'vue';
import { type scale } from './Point';

export class Area {
  id: number;
  name: string;
  leftTop: Position;
  rightBottom: Position;
  color: string;
  scale: scale;
  leftTopWaypoint: Position;
  rightBottomWaypoint: Position;
  constructor(id: number, name: string, leftTop: Position, rightBottom: Position, color: string, scale: scale) {
    this.id = id;
    this.name = name;
    this.leftTop = leftTop;
    this.rightBottom = rightBottom;
    this.color = color;
    this.scale = reactive({
      x: scale.x,
      y: scale.y
    });
    this.leftTopWaypoint = reactive({
      x: -1,
      y: -1
    });
    this.rightBottomWaypoint = reactive({
      x: -1,
      y: -1
    });
  }
}

// 旧接口保留兼容
export interface AreaInterface {
  id: number;
  name: string;
  leftTop: Position;
  rightBottom: Position;
  color: string;
}

