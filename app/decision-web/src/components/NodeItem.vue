<template>
  <button
    v-for="node in nodeList"
    :key="node.id.value"
    :data-id="`Node_${node.id.value}`"
    class="node"
    :class="{
      'dragging': (Node as any).draggingNodeId.value === node.id.value,
      'start-node': selectedStartNodeId === node.id.value,
      'end-node': selectedEndNodeId === node.id.value,
      'path-node': path_node.includes(node.id.value),
    }"
    :style="{
      width: `${node.shape.width}px`,
      height: `${node.shape.height}px`,
      position: 'absolute',
      left: `${node.position.x}px`,
      top: `${node.position.y}px`,
      borderRadius: `${node.shape.borderRadius}px`,
      border: `${node.shape.borderWidth}px solid ${node.color.borderColor}`,
      backgroundColor: node.color.fillColor,
      opacity: node.color.fillOpacity,
      fontSize: `${node.text.size}px`,
      color: node.text.color,
      fontFamily: node.text.fontFamily
    }"
    @mousedown="handleNodeMouseDown($event, node)"
  >
    <div class="node-icon" :style="{ backgroundColor: node.icon?.bgColor || '#3B82F6' }" v-if="node.icon">
      <svg v-if="node.icon.svgPath" class="icon-svg" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
        <path :d="node.icon.svgPath" fill="currentColor" />
      </svg>
      <span v-else-if="node.icon.text">{{ node.icon.text }}</span>
      <img v-else-if="node.icon.src" :src="node.icon.src" alt="icon" />
    </div>
    <div class="node-content">{{ node.text.content }}</div>

    <!-- 节点标记 -->
    <div class="node-marker" v-if="getNodeMarker(node)" :style="{ backgroundColor: getNodeMarker(node)?.color }">
      {{ getNodeMarker(node)?.label }}
    </div>

    <!-- 添加决策路径节点标记 -->
    <div class="path-node-marker" v-if="path_node.includes(node.id.value)">决策路径节点</div>
  </button>
</template>

<script setup lang="ts" name="NodeItem">
import { Node } from '@/types/Node';
import { activeToolId } from '@/types/ToolMenu';
import {
  isAddingEdge,
  handleNodeClick,
  selectedStartNodeId,
  selectedEndNodeId,
  renderNodeMarker as getNodeMarker
} from '@/types/extensions/ToolMenu/AddEdge';
import { nodes as nodeList } from '@/types/Manger';
import { ref } from 'vue';
import { path_node } from '@/types/extensions/Debug/debug';
// 获取NodeBase上的startDrag方法类型
interface NodeBaseMethods {
  startDrag: (event: MouseEvent, node: Node, nodeList: Node[]) => void;
  draggingNodeId: { value: number | null };
}

// 长按检测相关变量
const longPressTimeout = ref<number | null>(null);
const longPressDuration = 500; // 长按时间阈值，单位毫秒
const isDragging = ref(false);
const initialMousePosition = ref({ x: 0, y: 0 });
const mouseMoveThreshold = 5; // 鼠标移动阈值，用于区分点击和拖拽

// 处理节点鼠标按下事件
const handleNodeMouseDown = (event: MouseEvent, node: Node) => {
  // 检查当前是否处于添加连接线模式
  if (activeToolId.value === 'renderNodeEdge' && isAddingEdge.value) {
    // 如果是添加连接线模式，处理节点点击
    handleNodeClick(node.id.value, nodeList.value);
    // 阻止事件传播，避免触发拖拽行为
    event.stopPropagation();
    return;
  }

  // 记录初始鼠标位置
  initialMousePosition.value = { x: event.clientX, y: event.clientY };
  isDragging.value = false;

  // 设置长按定时器
  longPressTimeout.value = window.setTimeout(() => {
    // 长按触发，打开节点编辑菜单
    longPressTimeout.value = null;

  }, longPressDuration);

  // 添加鼠标移动和抬起事件监听
  const handleMouseMove = (e: MouseEvent) => {
    // 检查鼠标移动距离是否超过阈值
    const distance = Math.sqrt(
      Math.pow(e.clientX - initialMousePosition.value.x, 2) +
      Math.pow(e.clientY - initialMousePosition.value.y, 2)
    );

    if (distance > mouseMoveThreshold) {
      // 超过阈值，取消长按定时器
      if (longPressTimeout.value !== null) {
        clearTimeout(longPressTimeout.value);
        longPressTimeout.value = null;
        (Node as unknown as NodeBaseMethods).startDrag(event, node, nodeList.value);
      }
      if (!isDragging.value) {
        isDragging.value = true;
      }
    }
  };

  const handleMouseUp = () => {
    // 清除事件监听
    window.removeEventListener('mousemove', handleMouseMove);
    window.removeEventListener('mouseup', handleMouseUp);

    // 如果长按定时器仍存在，说明是短按（点击）
    if (longPressTimeout.value !== null) {
      clearTimeout(longPressTimeout.value);
      longPressTimeout.value = null;

      // 如果没有拖拽，则视为普通点击，可以在这里处理点击事件
      if (!isDragging.value) {
        // 普通点击操作，可以不执行任何操作，或者执行其他点击相关逻辑
        console.log('节点点击:', node.id.value);
        emit('open-edit-menu', node.id.value);
      }
    }
  };

  // 添加事件监听
  window.addEventListener('mousemove', handleMouseMove);
  window.addEventListener('mouseup', handleMouseUp);

  // 阻止默认行为
  event.preventDefault();
};

// 添加 emit 定义
const emit = defineEmits(['open-edit-menu']);

</script>

<style scoped>
.node {
  display: flex;
  align-items: center;
  cursor: grab;
  user-select: none;
  transition: all 0.2s ease;
  box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.1);
  text-align: left;
  font-weight: 500;
  overflow: hidden;
  padding: 0;
  outline: none;
  background-color: white;
  z-index: 1;
  position: relative;
}

.node.dragging {
  cursor: grabbing;
  opacity: 0.8;
  box-shadow: 0 8px 16px -2px rgba(0, 0, 0, 0.1), 0 4px 8px -2px rgba(0, 0, 0, 0.05);
  z-index: 100;
  transition: none;
}

.node.start-node {
  box-shadow: 0 0 0 2px #3B82F6; /* 蓝色边框 */
}

.node.end-node {
  box-shadow: 0 0 0 2px #EF4444; /* 红色边框 */
}

.node.path-node {
  box-shadow: 0 0 0 2px #10B981; /* 绿色边框 */
}

.node-icon {
  height: 100%;
  width: 40px;
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  font-weight: bold;
  font-size: 16px;
}

.node-icon img {
  max-width: 20px;
  max-height: 20px;
}

.icon-svg {
  width: 20px;
  height: 20px;
  stroke-width: 2;
  stroke: currentColor;
  fill: none;
}

.node-content {
  flex: 1;
  padding: 0 16px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.node:hover:not(.dragging) {
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
  transform: translateY(-1px);
}

.node:active:not(.dragging) {
  cursor: grabbing;
  transform: translateY(0);
  box-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
}

.node-marker {
  position: absolute;
  bottom: 4px;
  right: 4px;
  font-size: 12px;
  color: white;
  padding: 2px 6px;
  border-radius: 4px;
  font-weight: bold;
}

.path-node-marker {
  position: absolute;
  bottom: 4px;
  right: 4px;
  font-size: 12px;
  color: white;
  padding: 2px 6px;
  border-radius: 4px;
  font-weight: bold;
  background-color: #10B981; /* 绿色背景与边框匹配 */
}
</style>
