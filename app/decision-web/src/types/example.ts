import { Node } from '@/types/Node';
import { Edge } from '@/types/Edge';
import { nodes, edges } from '@/types/Manger';

export function example() {
  // 创建示例节点
  // 创建示例节点1 - 任务节点（带SVG图标 - 任务图标）
  const node1 = new Node(
    { value: 0 }, // id
    { x: 300, y: 200 }, // position
    { width: 200, height: 60, borderWidth: 1, borderRadius: 8 }, // shape
    { borderColor: '#E2E8F0', fillColor: '#FFFFFF', fillOpacity: 1 }, // color
    { size: 14, color: '#334155', content: '开始节点', fontFamily: 'Inter, system-ui, sans-serif' }, // text
    null, // taskConfig
    {
      svgPath: 'M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z',
      bgColor: '#10B981'
    } // icon - 圆圈图标
  );

  // 创建示例节点2 - 任务节点（带SVG图标 - 任务图标）
  const node2 = new Node(
    { value: 1 }, // id
    { x: 300, y: 350 }, // position
    { width: 200, height: 60, borderWidth: 1, borderRadius: 8 }, // shape
    { borderColor: '#E2E8F0', fillColor: '#FFFFFF', fillOpacity: 1 }, // color
    { size: 14, color: '#334155', content: '任务节点', fontFamily: 'Inter, system-ui, sans-serif' }, // text
    null, // taskConfig
    {
      svgPath: 'M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2m-6 9l2 2 4-4',
      bgColor: '#3B82F6'
    } // icon - 任务清单图标
  );

  // 创建示例节点3 - 决策节点（带SVG图标 - 菱形图标）
  const node3 = new Node(
    { value: 2 }, // id
    { x: 600, y: 350 }, // position
    { width: 200, height: 60, borderWidth: 1, borderRadius: 8 }, // shape
    { borderColor: '#FEE2E2', fillColor: '#FFFFFF', fillOpacity: 1 }, // color
    { size: 14, color: '#334155', content: '决策节点', fontFamily: 'Inter, system-ui, sans-serif' }, // text
    null, // taskConfig
    {
      svgPath: 'M4.5 15.75l7.5-7.5 7.5 7.5',
      bgColor: '#EF4444'
    } // icon - 菱形/箭头图标
  );

  // 创建示例节点4 - 结束节点（SVG圆圈图标）
  const node4 = new Node(
    { value: 3 }, // id
    { x: 600, y: 500 }, // position
    { width: 200, height: 60, borderWidth: 1, borderRadius: 8 }, // shape
    { borderColor: '#DCF5E8', fillColor: '#FFFFFF', fillOpacity: 1 }, // color
    { size: 14, color: '#334155', content: '结束节点', fontFamily: 'Inter, system-ui, sans-serif' }, // text
    null, // taskConfig
    {
      svgPath: 'M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z',
      bgColor: '#F97316'
    } // icon - 圆圈图标
  );

  // 添加节点到节点列表
  nodes.value = [node1, node2, node3, node4];

  // 创建示例边
  // 开始节点 -> 任务节点
  const edge1 = new Edge(
    { value: 0 },
    0, // 开始节点ID
    1, // 任务节点ID
    {
      color: '#10B981',
      width: 2,
      type: 'curve',
      curvature: 0.5,
      label: '开始流程'
    }
  );

  // 任务节点 -> 决策节点
  const edge2 = new Edge(
    { value: 1 },
    1, // 任务节点ID
    2, // 决策节点ID
    {
      color: '#3B82F6',
      width: 2,
      type: 'curve',
      curvature: 0.5
    }
  );

  // 决策节点 -> 结束节点
  const edge3 = new Edge(
    { value: 2 },
    2, // 决策节点ID
    3, // 结束节点ID
    {
      color: '#EF4444',
      width: 2,
      type: 'curve',
      curvature: 0.4,
      label: '结束流程'
    }
  );

  // 添加边到边列表
  edges.value = [edge1, edge2, edge3];
}