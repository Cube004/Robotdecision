import {
  outMapSettingsPoints,
  outNodes,
  outEdges,
  outPoints,
  outAreas,
  outNodeGroups,
  LoadMapSettingsPoints
} from './Object';
import { nodes, edges, points, areas, NodeGroups } from '@/types/Manger';
import { Node } from '@/types/Node';
import { Edge } from '@/types/Edge';
import { Point } from '@/types/Point';
import { Area } from '@/types/Area';
import type { Condition } from '@/types/Condition';

// 定义导入数据的类型接口
interface NodeData {
  id: number;
  position: { x: number; y: number };
  shape: {
    width: number;
    height: number;
    borderWidth: number;
    borderRadius: number
  };
  color: {
    borderColor: string;
    fillColor: string;
    fillOpacity: number;
  };
  text: {
    size: number;
    color: string;
    content: string;
    fontFamily: string;
  };
  taskConfig: {
    nodeType: string;
    mode: string;
    waypointId: number;
    spin: number;
    linear: number;
    resetTime: number;
  };
  icon?: {
    text?: string;
    bgColor?: string;
    src?: string;
    svgPath?: string;
  };
  edges: number[];
}

interface EdgeData {
  id: number;
  sourceId: number;
  targetId: number;
  config: {
    color: string;
    width: number;
    dashed: boolean;
    label?: string;
    type: 'straight' | 'curve';
    curvature?: number;
  };
  conditions: Condition[];
}

interface PointData {
  id: number;
  position: { x: number; y: number };
  waypoint: { x: number; y: number };
  color: string;
  text: string;
  fontSize: number;
  textColor: string;
  scale: { x: number; y: number };
}

interface AreaData {
  id: number;
  name: string;
  leftTop: { x: number; y: number };
  rightBottom: { x: number; y: number };
  color: string;
  scale: { x: number; y: number };
  leftTopWaypoint?: { x: number; y: number };
  rightBottomWaypoint?: { x: number; y: number };
}

interface NodeGroupData {
  id: number;
  name: string;
  color: string;
  nodesId: number[];
  config: {
    Loop: boolean;
    ResetTime: number;
    Reverse: boolean;
  };
}

interface RuleData {
  version: string;
  createdAt: string;
  mapSettings?: {
    points: PointData[];
  };
  nodes?: NodeData[];
  edges?: EdgeData[];
  points?: PointData[];
  areas?: AreaData[];
  nodeGroups?: NodeGroupData[];
}

/**
 * 导出完整的规则文件
 * 包含所有节点、边、点、区域、节点组和地图设置点
 */
export function exportRule() {
  // 获取各个部分的JSON数据（这里将字符串解析回对象）
  const mapSettingsPointsData = JSON.parse(outMapSettingsPoints());
  const nodesData = JSON.parse(outNodes());
  const edgesData = JSON.parse(outEdges());
  const pointsData = JSON.parse(outPoints());
  const areasData = JSON.parse(outAreas());
  const nodeGroupsData = JSON.parse(outNodeGroups());

  // 合并成一个完整的数据对象
  const completeData = {
    version: '1.0',
    createdAt: new Date().toISOString(),
    mapSettings: {
      points: mapSettingsPointsData
    },
    nodes: nodesData,
    edges: edgesData,
    points: pointsData,
    areas: areasData,
    nodeGroups: nodeGroupsData
  };

  // 将完整数据转换为格式化的JSON字符串
  const jsonData = JSON.stringify(completeData, null, 2);

  // 创建Blob对象
  const blob = new Blob([jsonData], { type: 'application/json' });
  const url = URL.createObjectURL(blob);

  // 创建下载链接
  const link = document.createElement('a');
  link.href = url;
  link.download = 'robot_decision_rule.json';
  document.body.appendChild(link);
  link.click();

  // 清理
  setTimeout(() => {
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  }, 100);

  return jsonData;
}

/**
 * 导入完整的规则文件
 */
export function importRule(fileData: string) {
  try {
    const data: RuleData = JSON.parse(fileData);

    // 导入地图设置点
    if (data.mapSettings?.points) {
      // 这里假设已经有处理地图设置点的相关函数
      LoadMapSettingsPoints();
    }

    // 导入节点
    if (data.nodes && Array.isArray(data.nodes)) {
      // 清空现有节点
      nodes.value = [];

      // 添加导入的节点
      data.nodes.forEach((nodeData: NodeData) => {
        const node = new Node(
          { value: nodeData.id }, // 使用对象形式包装ID以匹配NodeId类型
          nodeData.position,
          nodeData.shape,
          nodeData.color,
          nodeData.text,
          nodeData.taskConfig,
          nodeData.icon
        );
        nodes.value.push(node);
      });
    }

    // 导入边
    if (data.edges && Array.isArray(data.edges)) {
      // 清空现有边
      edges.value = [];

      // 添加导入的边
      data.edges.forEach((edgeData: EdgeData) => {
        const edge = new Edge(
          { value: edgeData.id },
          edgeData.sourceId,
          edgeData.targetId,
          edgeData.config
        );
        // 添加条件
        if (edgeData.conditions) {
          edge.conditions = edgeData.conditions;
        }
        edges.value.push(edge);
      });
    }

    // 导入点
    if (data.points && Array.isArray(data.points)) {
      // 清空现有点
      points.value = [];

      // 添加导入的点
      data.points.forEach((pointData: PointData) => {
        const point = new Point(
          pointData.id,
          pointData.position,
          pointData.waypoint,
          pointData.color,
          pointData.text,
          pointData.fontSize,
          pointData.textColor,
          pointData.scale
        );
        points.value.push(point);
      });
    }

    // 导入区域
    if (data.areas && Array.isArray(data.areas)) {
      // 清空现有区域
      areas.value = [];

      // 添加导入的区域
      data.areas.forEach((areaData: AreaData) => {
        const area = new Area(
          areaData.id,
          areaData.name,
          areaData.leftTop,
          areaData.rightBottom,
          areaData.color,
          areaData.scale
        );

        // 设置航点数据
        if (areaData.leftTopWaypoint) {
          area.leftTopWaypoint = areaData.leftTopWaypoint;
        }

        if (areaData.rightBottomWaypoint) {
          area.rightBottomWaypoint = areaData.rightBottomWaypoint;
        }

        areas.value.push(area);
      });
    }

    // 导入节点组
    if (data.nodeGroups && Array.isArray(data.nodeGroups)) {
      // 清空现有节点组
      NodeGroups.value = [];

      // 添加导入的节点组
      data.nodeGroups.forEach((groupData: NodeGroupData) => {
        NodeGroups.value.push({
          id: groupData.id,
          name: groupData.name,
          color: groupData.color,
          nodesId: groupData.nodesId,
          config: groupData.config
        });
      });
    }

    console.log('成功导入规则文件');
    return true;
  } catch (error) {
    console.error('导入规则文件失败:', error);
    return false;
  }
}

/**
 * 上传规则文件并导入
 */
export function uploadRuleFile() {
  // 创建文件输入元素
  const fileInput = document.createElement('input');
  fileInput.type = 'file';
  fileInput.accept = '.json';
  fileInput.style.display = 'none';
  document.body.appendChild(fileInput);

  // 设置文件选择事件
  fileInput.onchange = (event) => {
    const target = event.target as HTMLInputElement;
    const file = target.files?.[0];

    if (file) {
      const reader = new FileReader();

      reader.onload = (e) => {
        try {
          const fileData = e.target?.result as string;
          importRule(fileData);
        } catch (error) {
          console.error('读取文件失败:', error);
        }
      };

      reader.readAsText(file);
    }

    // 清理
    document.body.removeChild(fileInput);
  };

  // 触发文件选择对话框
  fileInput.click();
}
