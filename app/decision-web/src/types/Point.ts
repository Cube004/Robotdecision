import { reactive } from 'vue';

interface Id {
  value: number;
}

export interface Position {
  x: number;
  y: number;
}

interface Color {
  value: string;
}

interface Text {
  value: string;
}

interface FontSize {
  value: number;
}

interface TextColor {
  value: string;
}

export interface scale {
  x: number;
  y: number;
}
export class Point {
  id: Id;
  position: Position;
  waypoint: Position;
  color: Color;
  text: Text;
  fontSize: FontSize;
  textColor: TextColor;
  scale: scale;
  constructor(
    id: number, position: Position, waypoint: Position,
    color: string, text: string,
    fontSize: number, textColor: string,
    scale: scale
  ) {
    this.id = reactive({
      value: id
    });
    this.position = reactive({
      x: position.x,
      y: position.y
    });
    this.waypoint = reactive({
      x: waypoint.x,
      y: waypoint.y
    });
    this.color = reactive({
      value: color
    });
    this.text = reactive({
      value: text
    });
    this.fontSize = reactive({
      value: fontSize
    });
    this.textColor = reactive({
      value: textColor
    });
    this.scale = reactive({
      x: scale.x,
      y: scale.y
    });
  }
}