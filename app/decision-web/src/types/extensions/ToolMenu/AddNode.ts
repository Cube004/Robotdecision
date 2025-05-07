import { ref } from 'vue';
import { Node } from '@/types/Node';
import { nodes } from '@/types/Manger';
import { activeToolId } from '@/types/ToolMenu';
import { mousePositionCanvas } from '@/types/Manger';
import { GetNewId } from '@/types/Manger';
// 用于存储当前操作模式状态
export const isAddingNode = ref<boolean>(false);

// 存储鼠标位置
export const mousePosition = ref<{ x: number, y: number }>({ x: 0, y: 0 });

// 是否正在放置节点
export const isPlacingNode = ref<boolean>(false);

/**
 * 初始化添加节点状态
 */
export function initAddNodeState() {
  isAddingNode.value = true;
  isPlacingNode.value = false;
}

/**
 * 重置添加节点状态
 */
export function resetAddNodeState() {
  isAddingNode.value = false;
  isPlacingNode.value = false;
}

/**
 * 确认添加节点，进入放置模式
 */
export function handleConfirmAddNode() {
  console.log('确认添加节点');
  isPlacingNode.value = true;
}

/**
 * 取消添加节点
 */
export function handleCancelAddNode() {
  console.log('取消添加节点');
  resetAddNodeState();
  activeToolId.value = 'cursor';
}

/**
 * 处理放置节点
 */
export function handlePlaceNode(position: { x: number, y: number }) {
  console.log('添加新节点位置:', position);
  // 获取下一个节点ID
  const nextNodeId = GetNewId(nodes.value.map(node => node.id.value));
  // 创建新节点
  const newNode = createNode(position, nextNodeId);
  // 添加到节点列表
  nodes.value.push(newNode);
  // 重置工具状态
  resetAddNodeState();
  activeToolId.value = 'cursor';
};

/**
 * 更新鼠标位置
 * @param position 鼠标位置
 */
export function updateMousePosition(position: { x: number, y: number }) {
  mousePosition.value = position;
}

/**
 * 创建新节点
 * @param position 节点位置
 * @param nextNodeId 下一个节点的ID
 */
export function createNode(position: { x: number, y: number }, nextNodeId: number): Node {
  // 创建新节点
  const newNode = new Node(
    { value: nextNodeId }, // id
    { x: mousePositionCanvas.value.x - 100, y: mousePositionCanvas.value.y - 30 }, // 居中位置
    { width: 200, height: 60, borderWidth: 1, borderRadius: 8 }, // shape
    { borderColor: '#E2E8F0', fillColor: '#FFFFFF', fillOpacity: 1 }, // color
    { size: 14, color: '#334155', content: '任务节点', fontFamily: 'Inter, system-ui, sans-serif' }, // text
    null, // taskConfig
    {
      svgPath: 'M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2m-6 9l2 2 4-4',
      bgColor: '#3B82F6'
    } // icon - 任务清单图标
  );

  // 重置状态
  resetAddNodeState();

  return newNode;
}
