import { Point } from '@/types/Point';
import { Node } from '@/types/Node';
import { Edge } from '@/types/Edge';

import { nodes, edges, points } from '@/types/Manger';
import { MapSettingsPoints, mapWidth, mapHeight } from '@/types/Manger';

export {
  nodes,
  edges,
  points,
  MapSettingsPoints,
  mapWidth,
  mapHeight
}

// 返回JSON格式
export function outPoints(points: Point[]) {
  return JSON.stringify(points.map(point => ({
    id: point.id.value,
    position: {
      x: point.position.x,
      y: point.position.y
    },
    waypoint: {
      x: point.waypoint.x,
      y: point.waypoint.y
    },
    color: point.color.value,
    text: point.text.value,
    fontSize: point.fontSize.value,
    textColor: point.textColor.value,
    scale: {
      x: point.scale.x,
      y: point.scale.y
    }
  })));
}

// 返回JSON格式
export function outEdges(edges: Edge[]) {
  return JSON.stringify(edges.map(edge => ({
    id: edge.id.value,
    sourceId: edge.sourceId,
    targetId: edge.targetId,
    config: {
      color: edge.config.color,
      width: edge.config.width,
      dashed: edge.config.dashed,
      label: edge.config.label,
      type: edge.config.type,
      curvature: edge.config.curvature
    },
    conditions: edge.conditions
  })));
}

// 返回JSON格式
export function outNodes(nodes: Node[]) {
  return JSON.stringify(nodes.map(node => ({
    id: node.id.value,
    position: {
      x: node.position.x,
      y: node.position.y
    },
    shape: {
      width: node.shape.width,
      height: node.shape.height,
      borderWidth: node.shape.borderWidth,
      borderRadius: node.shape.borderRadius
    },
    color: {
      borderColor: node.color.borderColor,
      fillColor: node.color.fillColor,
      fillOpacity: node.color.fillOpacity
    },
    text: {
      size: node.text.size,
      color: node.text.color,
      content: node.text.content,
      fontFamily: node.text.fontFamily
    },
    taskConfig: {
      nodeType: node.taskConfig.nodeType,
      mode: node.taskConfig.mode,
      waypointId: node.taskConfig.waypointId,
      spin: node.taskConfig.spin,
      resetTime: node.taskConfig.resetTime
    },
    icon: {
      text: node.icon?.text,
      bgColor: node.icon?.bgColor,
      src: node.icon?.src,
      svgPath: node.icon?.svgPath
    }
  })));
}


