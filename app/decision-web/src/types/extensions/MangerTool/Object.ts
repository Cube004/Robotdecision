import { Point } from '@/types/Point';
import { nodes, edges, points, areas, NodeGroups } from '@/types/Manger';
import { MapSettingsPoints, mapWidth, mapHeight } from '@/types/Manger';

export {
  nodes,
  edges,
  points,
  MapSettingsPoints,
  mapWidth,
  mapHeight
}

export function outFile(){
  const jsonData = outMapSettingsPoints();
  const blob = new Blob([jsonData], { type: 'application/json' });
  const url = URL.createObjectURL(blob);

  const link = document.createElement('a');
  link.href = url;
  link.download = 'map_settings_points.json';
  document.body.appendChild(link);
  link.click();

  // 清理
  setTimeout(() => {
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  }, 100);
}

export function outMapSettingsPoints() {
  const Point = [...MapSettingsPoints.value];
  const json = Point.map(item => {
    return {
      id: item.id.value,
      position: {
        x: item.position.x,
        y: item.position.y
      },
      waypoint: {
        x: item.waypoint.x,
        y: item.waypoint.y
      },
      color: item.color.value,
      text: item.text.value,
      fontSize: item.fontSize.value,
      textColor: item.textColor.value,
      scale: {
        x: item.scale.x,
        y: item.scale.y
      }
    };
  });
  return JSON.stringify(json, null, 2);
}

export function inFile() {
  LoadMapSettingsPoints();
}

export function LoadMapSettingsPoints() {
  fetch('/rules/map_settings_points.json')
    .then(response => {
      if (!response.ok) {
        throw new Error('无法加载文件: ' + response.statusText);
      }
      return response.json();
    })
    .then(data => {
      // 清空当前数据
      MapSettingsPoints.value = [];

      // 将JSON数据转换为Point对象
      if (Array.isArray(data)) {
        data.forEach(item => {
          const point = new Point(
            item.id,
            item.position,
            item.waypoint,
            item.color,
            item.text,
            item.fontSize,
            item.textColor,
            item.scale
          );
          MapSettingsPoints.value.push(point);
        });
        console.log('成功加载地图设置点:', MapSettingsPoints.value.length);
      } else {
        console.error('无效的JSON数据格式');
      }
    })
    .catch(error => {
      console.error('加载文件时出错:', error);
    });
}
LoadMapSettingsPoints();
export function outNodes() {
  const nodeList = [...nodes.value];
  const json = nodeList.map(item => {
    return {
      id: item.id.value,
      position: {
        x: item.position.x,
        y: item.position.y
      },
      shape: {
        width: item.shape.width,
        height: item.shape.height,
        borderWidth: item.shape.borderWidth,
        borderRadius: item.shape.borderRadius
      },
      color: {
        borderColor: item.color.borderColor,
        fillColor: item.color.fillColor,
        fillOpacity: item.color.fillOpacity
      },
      text: {
        size: item.text.size,
        color: item.text.color,
        content: item.text.content,
        fontFamily: item.text.fontFamily
      },
      taskConfig: {
        nodeType: item.taskConfig.nodeType,
        mode: item.taskConfig.mode,
        waypointId: item.taskConfig.waypointId,
        spin: item.taskConfig.spin,
        linear: item.taskConfig.linear,
        resetTime: item.taskConfig.resetTime
      },
      icon: {
        text: item.icon?.text,
        bgColor: item.icon?.bgColor,
        src: item.icon?.src,
        svgPath: item.icon?.svgPath
      },
      edges: item.edges
    };
  });
  return JSON.stringify(json, null, 2);
}

export function outEdges() {
  const edgeList = [...edges.value];
  const json = edgeList.map(item => {
    return {
      id: item.id.value,
      sourceId: item.sourceId,
      targetId: item.targetId,
      config: {
        color: item.config.color,
        width: item.config.width,
        dashed: item.config.dashed,
        label: item.config.label,
        type: item.config.type,
        curvature: item.config.curvature
      },
      conditions: item.conditions
    };
  });
  return JSON.stringify(json, null, 2);
}

export function outPoints() {
  const pointList = [...points.value];
  const json = pointList.map(item => {
    return {
      id: item.id.value,
      position: {
        x: item.position.x,
        y: item.position.y
      },
      waypoint: {
        x: item.waypoint.x,
        y: item.waypoint.y
      },
      color: item.color.value,
      text: item.text.value,
      fontSize: item.fontSize.value,
      textColor: item.textColor.value,
      scale: {
        x: item.scale.x,
        y: item.scale.y
      }
    };
  });
  return JSON.stringify(json, null, 2);
}

export function outAreas() {
  const areaList = [...areas.value];
  const json = areaList.map(item => {
    return {
      id: item.id,
      name: item.name,
      leftTop: item.leftTop,
      rightBottom: item.rightBottom,
      color: item.color,
      scale: item.scale,
      leftTopWaypoint: item.leftTopWaypoint,
      rightBottomWaypoint: item.rightBottomWaypoint
    };
  });
  return JSON.stringify(json, null, 2);
}

export function outNodeGroups() {
  const groupList = [...NodeGroups.value];
  const json = groupList.map(item => {
    return {
      id: item.id,
      name: item.name,
      color: item.color,
      nodesId: item.nodesId,
      config: {
        Loop: item.config.Loop,
        ResetTime: item.config.ResetTime,
        Reverse: item.config.Reverse
      }
    };
  });
  return JSON.stringify(json, null, 2);
}
