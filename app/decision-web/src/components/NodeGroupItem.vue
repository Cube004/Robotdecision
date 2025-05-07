<template>
  <div
    v-for="group in nodeGroups"
    :key="group.id"
    class="node-group"
    :style="{
      position: 'absolute',
      left: `${groupPositions[group.id]?.x || 0}px`,
      top: `${groupPositions[group.id]?.y || 0}px`,
      width: `${groupPositions[group.id]?.width || 0}px`,
      height: `${groupPositions[group.id]?.height || 0}px`,
      backgroundColor: group.color,
      borderRadius: '8px',
      border: `2px solid ${getGroupBorderColor(group.color)}`,
      zIndex: 0,
      cursor: isDragging === group.id ? 'grabbing' : 'grab'
    }"
    @mousedown="handleGroupMouseDown($event, group)"
  >
    <div
      class="group-label"
      :style="{ backgroundColor: getGroupBorderColor(group.color) }"
    >
      {{ group.name }} (#{{ group.id }})
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, ref, onMounted, watch } from 'vue';
import { nodes, NodeGroups, mousePositionCanvas } from '@/types/Manger';

// 节点组的位置和大小
interface GroupPosition {
  x: number;
  y: number;
  width: number;
  height: number;
}

// 节点组位置映射
const groupPositions = ref<Record<number, GroupPosition>>({});

// 监听节点组变化
const nodeGroups = computed(() => NodeGroups.value);

// 正在拖拽的组ID
const isDragging = ref<number | null>(null);
// 拖拽起始点
const dragStartPos = ref({ x: 0, y: 0 });
// 拖拽起始组位置
const dragStartGroupPos = ref<GroupPosition | null>(null);
// 拖拽中的节点列表
const dragNodesStartPos = ref<Record<number, { x: number, y: number }>>({});

// 获取边框颜色（比背景色深）
const getGroupBorderColor = (color: string) => {
  try {
    if (color.startsWith('rgba')) {
      // 处理rgba格式
      const parts = color.match(/rgba\((\d+),\s*(\d+),\s*(\d+),\s*([\d.]+)\)/);
      if (parts) {
        const [, r, g, b, a] = parts;
        const darkenFactor = 0.3;
        const darkenR = Math.max(0, Math.floor(Number(r) * (1 - darkenFactor)));
        const darkenG = Math.max(0, Math.floor(Number(g) * (1 - darkenFactor)));
        const darkenB = Math.max(0, Math.floor(Number(b) * (1 - darkenFactor)));
        return `rgba(${darkenR}, ${darkenG}, ${darkenB}, ${a})`;
      }
    }

    // 处理hex格式
    if (color.startsWith('#')) {
      const hex = color.replace('#', '');
      const r = parseInt(hex.substring(0, 2), 16);
      const g = parseInt(hex.substring(2, 4), 16);
      const b = parseInt(hex.substring(4, 6), 16);
      // 提取透明度，如果存在
      let opacity = 1.0;
      if (hex.length >= 8) {
        opacity = parseInt(hex.substring(6, 8), 16) / 255;
      }
      // 减少亮度，但不低于0
      const darkenFactor = 0.3;
      const darkenR = Math.max(0, Math.floor(r * (1 - darkenFactor)));
      const darkenG = Math.max(0, Math.floor(g * (1 - darkenFactor)));
      const darkenB = Math.max(0, Math.floor(b * (1 - darkenFactor)));
      // 返回rgba格式
      return `rgba(${darkenR}, ${darkenG}, ${darkenB}, ${opacity})`;
    }

    return '#666666'; // 默认颜色
  } catch {
    return '#666666'; // 默认颜色
  }
};

// 计算每个组的位置和尺寸
const calculateGroupPositions = () => {
  const positions: Record<number, GroupPosition> = {};

  nodeGroups.value.forEach(group => {
    // 获取该组包含的所有节点
    const groupNodes = nodes.value.filter(node => group.nodesId.includes(node.id.value));

    if (groupNodes.length > 0) {
      // 找出最左、最上、最右、最下的节点坐标
      let minX = Number.MAX_VALUE;
      let minY = Number.MAX_VALUE;
      let maxX = Number.MIN_VALUE;
      let maxY = Number.MIN_VALUE;

      groupNodes.forEach(node => {
        const { x, y } = node.position;
        const right = x + node.shape.width;
        const bottom = y + node.shape.height;

        minX = Math.min(minX, x);
        minY = Math.min(minY, y);
        maxX = Math.max(maxX, right);
        maxY = Math.max(maxY, bottom);
      });

      // 添加padding
      const padding = 20;
      positions[group.id] = {
        x: minX - padding,
        y: minY - padding,
        width: maxX - minX + (padding * 2),
        height: maxY - minY + (padding * 2),
      };
    }
  });

  groupPositions.value = positions;
};

// 处理组鼠标按下事件（开始拖拽）
const handleGroupMouseDown = (event: MouseEvent, group: typeof nodeGroups.value[0]) => {
  // 阻止事件冒泡，防止触发canvas的事件
  event.stopPropagation();
  event.preventDefault();

  // 记录拖拽起始点
  dragStartPos.value = { x: mousePositionCanvas.value.x, y: mousePositionCanvas.value.y };
  isDragging.value = group.id;
  dragStartGroupPos.value = { ...groupPositions.value[group.id] };

  // 记录组内所有节点的起始位置
  dragNodesStartPos.value = {};
  group.nodesId.forEach(nodeId => {
    const node = nodes.value.find(n => n.id.value === nodeId);
    if (node) {
      dragNodesStartPos.value[nodeId] = {
        x: node.position.x,
        y: node.position.y
      };
    }
  });

  // 添加鼠标移动和抬起事件
  window.addEventListener('mousemove', handleMouseMove);
  window.addEventListener('mouseup', handleMouseUp);
};

// 处理鼠标移动事件（拖拽中）
const handleMouseMove = () => {
  if (isDragging.value !== null && dragStartGroupPos.value) {
    // 计算移动距离
    const deltaX = mousePositionCanvas.value.x - dragStartPos.value.x;
    const deltaY = mousePositionCanvas.value.y - dragStartPos.value.y;
    // 同步更新组位置和节点位置
    const group = nodeGroups.value.find(g => g.id === isDragging.value);
    if (group) {

      const nodesToUpdate = nodes.value.filter(n => group.nodesId.includes(n.id.value));
      nodesToUpdate.forEach(node => {
        const startPos = dragNodesStartPos.value[node.id.value];
        if (startPos) {
          node.position = {
            x: startPos.x + deltaX,
            y: startPos.y + deltaY
          };
        }
      });
      // 然后更新组位置
      groupPositions.value[isDragging.value] = {
        ...dragStartGroupPos.value,
        x: mousePositionCanvas.value.x - groupPositions.value[isDragging.value].width / 2,
        y: mousePositionCanvas.value.y - groupPositions.value[isDragging.value].height / 2
      };
    }
  }
};

// 处理鼠标抬起事件（结束拖拽）
const handleMouseUp = () => {
  isDragging.value = null;

  // 移除事件监听
  window.removeEventListener('mousemove', handleMouseMove);
  window.removeEventListener('mouseup', handleMouseUp);
};

// 监听节点变化，重新计算组位置
watch([nodes, nodeGroups], () => {
  calculateGroupPositions();
}, { deep: true });

// 组件挂载时初始化
onMounted(() => {
  calculateGroupPositions();
});
</script>

<style scoped>
.node-group {
  pointer-events: all;
  transition: opacity 0.2s ease;
}

.node-group:hover {
  opacity: 0.7 !important;
}

.group-label {
  position: absolute;
  top: -30px;
  right: 10px;
  padding: 4px 8px;
  border-radius: 4px;
  color: white;
  font-size: 12px;
  font-weight: bold;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  z-index: 100;
}
</style>
