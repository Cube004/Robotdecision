import { ref, watch } from 'vue';
import { Node, NodeType } from '@/types/Node';
import { Edge } from '@/types/Edge';
import { activeToolId } from '@/types/ToolMenu';
import { edges } from '@/types/Manger';
import type { EdgeId } from '@/types/EdgeBase';
import { showError, GetNewId } from '@/types/Manger';
// 节点选择状态
export const selectedStartNodeId = ref<number | null>(null);
export const selectedEndNodeId = ref<number | null>(null);
export const isAddingEdge = ref<boolean>(false);

// 节点上的标记样式
interface NodeMarker {
  nodeId: number;
  label: string;
  color: string;
}

// 存储添加的节点标记
export const nodeMarkers = ref<NodeMarker[]>([]);

/**
 * 初始化添加连接线状态
 */
export function initAddEdgeState() {
  isAddingEdge.value = true;
  selectedStartNodeId.value = null;
  selectedEndNodeId.value = null;
  nodeMarkers.value = [];
}

/**
 * 重置添加连接线状态
 */
export function resetAddEdgeState() {
  isAddingEdge.value = false;
  selectedStartNodeId.value = null;
  selectedEndNodeId.value = null;
  nodeMarkers.value = [];
}

/**
 * 选择起点节点
 * @param nodeId 节点ID
 */
export function selectStartNode(nodeId: number) {
  selectedStartNodeId.value = nodeId;
  // 添加起点标记
  nodeMarkers.value.push({
    nodeId,
    label: '起点',
    color: '#3B82F6' // 蓝色
  });
}

/**
 * 选择终点节点
 * @param nodeId 节点ID
 */
export function selectEndNode(nodeId: number) {
  if (nodeId === selectedStartNodeId.value) {
    return; // 不允许起点和终点相同
  }

  selectedEndNodeId.value = nodeId;
  // 添加终点标记
  nodeMarkers.value.push({
    nodeId,
    label: '终点',
    color: '#EF4444' // 红色
  });
}

/**
 * 处理节点点击事件
 * @param nodeId 被点击的节点ID
 * @param nodes 所有节点列表
 */
export function handleNodeClick(nodeId: number, nodes: Node[]) {
  if (!isAddingEdge.value) return;

  const clickedNode = nodes.find(node => node.id.value === nodeId);
  if (!clickedNode) return;

  if (selectedStartNodeId.value === null) {
    // 选择起点
    if (clickedNode.taskConfig.nodeType === NodeType.Task) {
      showError('任务节点不能作为起点');
    }else{
      selectStartNode(nodeId);
    }
  } else if (selectedEndNodeId.value === null) {
    // 选择终点
    selectEndNode(nodeId);
  }
}

/**
 * 根据节点ID获取节点对象
 * @param nodeId 节点ID
 * @param nodes 所有节点列表
 */
export function getNodeById(nodeId: number, nodes: Node[]): Node | undefined {
  return nodes.find(node => node.id.value === nodeId);
}

/**
 * 创建新连接线
 * @param edgeId 连接线ID
 */
export function createEdge(edgeId: EdgeId): Edge | null {
  if (selectedStartNodeId.value === null || selectedEndNodeId.value === null) {
    return null;
  }

  // 创建新连接线
  const newEdge = new Edge(
    edgeId,
    selectedStartNodeId.value,
    selectedEndNodeId.value,
    {
      color: '#3B82F6', // 蓝色
      width: 2,
      type: 'curve',
      curvature: 0.5
    }
  );

  // 重置状态
  resetAddEdgeState();

  return newEdge;
}

/**
 * 高亮显示节点
 * @param nodeId 节点ID
 * @param highlight 是否高亮
 * @param color 高亮颜色
 */
export function highlightNode(nodeId: number, highlight: boolean, color: string = '#3B82F6') {
  // 实现节点高亮逻辑，这可能涉及到DOM操作或状态管理
  const nodeElement = document.querySelector(`[data-id="Node_${nodeId}"]`) as HTMLElement;

  if (nodeElement) {
    if (highlight) {
      // 添加高亮效果
      nodeElement.style.boxShadow = `0 0 0 2px ${color}`;
    } else {
      // 移除高亮效果
      nodeElement.style.boxShadow = '';
    }
  }
}

// 监听起点和终点选择状态的变化
watch(selectedStartNodeId, (newVal, oldVal) => {
  if (oldVal !== null) {
    // 移除旧起点的高亮
    highlightNode(oldVal, false);
  }

  if (newVal !== null) {
    // 高亮新起点
    highlightNode(newVal, true, '#3B82F6');
  }
});

watch(selectedEndNodeId, (newVal, oldVal) => {
  if (oldVal !== null) {
    // 移除旧终点的高亮
    highlightNode(oldVal, false);
  }

  if (newVal !== null) {
    // 高亮新终点
    highlightNode(newVal, true, '#EF4444');
  }
});

// 监听工具选择状态变化
watch(activeToolId, (newVal) => {
  if (newVal !== 'renderNodeEdge') {
    // 如果切换到其他工具，重置添加连接线状态
    resetAddEdgeState();
  }
});

/**
 * 渲染节点标记
 * @param node 节点对象
 */
export function renderNodeMarker(node: Node) {
  const marker = nodeMarkers.value.find(m => m.nodeId === node.id.value);
  if (!marker) return null;

  return {
    label: marker.label,
    color: marker.color
  };
}


/**
 * 确认添加连接线
 * @param startNodeId 起点节点ID
 * @param endNodeId 终点节点ID
 */
export function handleConfirmAddEdge({ startNodeId, endNodeId }: { startNodeId: number, endNodeId: number }) {
  console.log('确认添加连接线', startNodeId, endNodeId);

  // 生成唯一ID
  const edgeId = { value: GetNewId(edges.value.map(edge => edge.id.value)) };
  // 创建新连接线
  const newEdge = createEdge(edgeId);

  console.log("使用工具栏添加的边:", newEdge?.id.value);
  console.log("当前所有边的id:", edges.value.map(edge => edge.id.value));
  // 重置状态
  resetAddEdgeState();
  // 切换回选择工具
  activeToolId.value = 'cursor';

  // 添加边
  if (newEdge) {
    for (const edge of edges.value) {
      if (edge.sourceId === startNodeId && edge.targetId === endNodeId) {
        showError('不能添加重复的边');
        return;
      }
      if (edge.sourceId === endNodeId && edge.targetId === startNodeId) {
        showError('不能添加反向的边');
        return;
      }
    }
    console.log('可以添加边');
    edges.value.push(newEdge);
  }
};

/**
 *  取消添加连接线
 */
export function handleCancelAddEdge(){
  console.log('取消添加连接线');

  // 重置状态
  resetAddEdgeState();
  // 切换回选择工具
  activeToolId.value = 'cursor';
};