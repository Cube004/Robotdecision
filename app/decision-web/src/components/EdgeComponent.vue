<template>
  <svg class="edge-layer" :width="svgWidth" :height="svgHeight">
    <!-- 定义箭头标记 -->
    <defs>
      <marker
        v-for="edge in edgeList"
        :key="`marker-${edge.id.value}`"
        :id="`arrow-${edge.id.value}`"
        viewBox="0 0 10 10"
        refX="5"
        refY="5"
        markerWidth="6"
        markerHeight="6"
        orient="auto-start-reverse"
      >
        <path d="M 0 0 L 10 5 L 0 10 z" :fill="edge.config.color" />
      </marker>
    </defs>

    <!-- 绘制连接线路径 -->
    <g v-for="edge in edgeList" :key="`edge-group-${edge.id.value}`">
      <path
        :d="calculatePath(edge)"
        :stroke="edge.config.color"
        :stroke-width="edge.config.width"
        :stroke-dasharray="edge.config.dashed || edge.conditions.length == 0? '5,5' : 'none'"
        :marker-end="`url(#arrow-${edge.id.value})`"
        fill="none"
        class="edge-path"
        @click="(e) => handleEdgeClick(edge, e)"
        @mouseover="() => handleEdgeHover(edge, true)"
        @mouseleave="() => handleEdgeHover(edge, false)"
        :class="{ 'hovered': hoveredEdgeId === edge.id.value }"
      />

      <!-- 为连接线添加透明但更宽的路径用于更容易点击 -->
      <path
        :d="calculatePath(edge)"
        stroke="transparent"
        stroke-width="15"
        fill="none"
        style="pointer-events: all; cursor: pointer;"
        @click="(e) => handleEdgeClick(edge, e)"
        @mouseover="() => handleEdgeHover(edge, true)"
        @mouseleave="() => handleEdgeHover(edge, false)"
      />
    </g>

    <!-- 连接线标签 -->
    <text
      v-for="edge in edgesWithLabels"
      :key="`label-${edge.id.value}`"
      :x="getEdgeCenter(edge).x"
      :y="getEdgeCenter(edge).y - 5"
      text-anchor="middle"
      dominant-baseline="auto"
      class="edge-label"
      :class="{ 'hovered': hoveredEdgeId === edge.id.value }"
    >
      {{ edge.config.label }}
    </text>

  </svg>
</template>

<script setup lang="ts">
import { computed, ref, onMounted, onUnmounted } from 'vue';
import Edge from '@/types/EdgeBase';
import { nodes, edges } from '@/types/Manger';
interface Props {
  // 箭头与节点边界的偏移距离
  arrowOffset?: number;
  // 同一直线的容差值（像素）
  alignmentTolerance?: number;
}

const props = withDefaults(defineProps<Props>(), {
  edges: () => [],
  arrowOffset: 15,  // 默认偏移15像素
  alignmentTolerance: 10  // 默认容差10像素
});

const emit = defineEmits(['edge-click']);

const edgeList = computed(() => edges.value);
const edgesWithLabels = computed(() => edges.value.filter(edge => edge.config.label));

const svgWidth = ref(window.innerWidth);
const svgHeight = ref(window.innerHeight);
const hoveredEdgeId = ref<number | null>(null);
const clickedEdgeId = ref<number | null>(null);
const clickedPosition = ref({ x: 0, y: 0 });

// 处理连接线点击
const handleEdgeClick = (edge: Edge, event?: MouseEvent) => {
  event = event || window.event as MouseEvent;
  clickedEdgeId.value = edge.id.value;
  // 设置弹窗位置在鼠标点击位置或连接线中心
  if (event) {
    clickedPosition.value = { x: event.offsetX, y: event.offsetY - 40 }; // 上方偏移避免被鼠标遮挡
  } else {
    const center = getEdgeCenter(edge);
    clickedPosition.value = { x: center.x, y: center.y - 40 };
  }
  console.log(`点击了连接线: ${edge.id.value}`);
  emit('edge-click', edge.id.value);
};

// 处理连接线悬停
const handleEdgeHover = (edge: Edge, isHovering: boolean) => {
  hoveredEdgeId.value = isHovering ? edge.id.value : null;
};


// 监听窗口大小变化
const handleResize = () => {
  svgWidth.value = window.innerWidth;
  svgHeight.value = window.innerHeight;
};

onMounted(() => {
  window.addEventListener('resize', handleResize);
  // 点击SVG外部时关闭弹窗
  window.addEventListener('click', (e) => {
    if (!(e.target as Element).closest('.edge-path, .edge-tooltip')) {
      clickedEdgeId.value = null;
    }
  });
});

onUnmounted(() => {
  window.removeEventListener('resize', handleResize);
  window.removeEventListener('click', () => {});
});

// 计算节点的中心点坐标
const getNodeCenter = (nodeId: number) => {
  const node = nodes.value.find(n => n.id.value === nodeId);
  if (!node) return { x: 0, y: 0 };

  return {
    x: node.position.x + node.shape.width / 2,
    y: node.position.y + node.shape.height / 2
  };
};

// 计算角度（0-360度）
const calculateAngle = (fromX: number, fromY: number, toX: number, toY: number) => {
  const dx = toX - fromX;
  const dy = toY - fromY;

  // 计算角度（弧度）
  let angle = Math.atan2(dy, dx);

  // 转换为0-360度
  if (angle < 0) {
    angle += 2 * Math.PI;
  }

  // 转换为角度
  return angle * (180 / Math.PI);
};

// 计算节点边界上的点，考虑实际方向和偏移量
const getNodeBoundary = (nodeId: number, fromPoint: {x: number, y: number}) => {
  const node = nodes.value.find(n => n.id.value === nodeId);
  if (!node) return { x: 0, y: 0 };

  // 节点的中心点
  const center = {
    x: node.position.x + node.shape.width / 2,
    y: node.position.y + node.shape.height / 2
  };

  // 计算角度
  const angle = calculateAngle(fromPoint.x, fromPoint.y, center.x, center.y);

  // 节点的边界坐标
  const left = node.position.x;
  const top = node.position.y;
  const right = left + node.shape.width;
  const bottom = top + node.shape.height;

  // 设置偏移量
  const offset = props.arrowOffset;

  // 确定主要方向
  const mainDirection = getMainDirection(fromPoint, center);

  // 根据主要方向确定边界点
  switch (mainDirection) {
    case 'horizontal':
      // 水平方向主导，使用左/右边界
      if (fromPoint.x < center.x) {
        // 从左侧进入
        return { x: left - offset, y: center.y };
      } else {
        // 从右侧进入
        return { x: right + offset, y: center.y };
      }
    case 'vertical':
      // 垂直方向主导，使用上/下边界
      if (fromPoint.y < center.y) {
        // 从上方进入
        return { x: center.x, y: top - offset };
      } else {
        // 从下方进入
        return { x: center.x, y: bottom + offset };
      }
    default:
      // 默认基于角度计算
      if (angle >= 315 || angle < 45) {
        // 从左侧进入节点
        return { x: left - offset, y: center.y };
      } else if (angle >= 45 && angle < 135) {
        // 从上方进入节点
        return { x: center.x, y: top - offset };
      } else if (angle >= 135 && angle < 225) {
        // 从右侧进入节点
        return { x: right + offset, y: center.y };
      } else {
        // 从下方进入节点
        return { x: center.x, y: bottom + offset };
      }
  }
};

// 确定两点之间的主要方向（水平或垂直）
const getMainDirection = (point1: {x: number, y: number}, point2: {x: number, y: number}) => {
  const dx = Math.abs(point2.x - point1.x);
  const dy = Math.abs(point2.y - point1.y);

  // 如果水平距离明显大于垂直距离，则认为是水平方向
  if (dx > dy * 1.5) {
    return 'horizontal';
  }

  // 如果垂直距离明显大于水平距离，则认为是垂直方向
  if (dy > dx * 1.5) {
    return 'vertical';
  }

  // 否则不确定主方向，使用角度计算
  return 'diagonal';
};

// 计算连接线的中心点（用于标签定位）
const getEdgeCenter = (edge: Edge) => {
  const source = getNodeCenter(edge.sourceId);
  const target = getNodeCenter(edge.targetId);

  return {
    x: (source.x + target.x) / 2,
    y: (source.y + target.y) / 2
  };
};

// 计算连接线的路径
const calculatePath = (edge: Edge) => {
  const source = getNodeCenter(edge.sourceId);
  const targetCenter = getNodeCenter(edge.targetId);

  // 计算源节点和目标节点的边界点
  const sourcePoint = getNodeBoundary(edge.sourceId, targetCenter);
  const targetPoint = getNodeBoundary(edge.targetId, source);

  // 检查两个节点是否在同一水平线或垂直线上
  const isAligned = checkNodesAlignment(sourcePoint, targetPoint);

  // 如果节点在同一直线上，或者配置为直线类型，则使用直线连接
  if (isAligned || edge.config.type === 'straight') {
    // 直线连接
    return `M ${sourcePoint.x} ${sourcePoint.y} L ${targetPoint.x} ${targetPoint.y}`;
  } else {
    // 贝塞尔曲线连接
    // 计算主要方向
    const mainDirection = getMainDirection(source, targetCenter);
    const curvature = edge.config.curvature || 0.5;

    // 根据主要方向计算控制点
    let controlPoint1, controlPoint2;

    if (mainDirection === 'horizontal') {
      // 水平方向的曲线控制点
      const dx = targetPoint.x - sourcePoint.x;
      const midX = sourcePoint.x + dx * 0.5;

      // 根据曲率计算控制点的垂直偏移
      const vertOffset = Math.min(Math.abs(dx) * 0.3, 80) * curvature;
      const sign = sourcePoint.y > targetPoint.y ? -1 : 1; // 确定偏移方向

      controlPoint1 = {
        x: midX,
        y: sourcePoint.y + vertOffset * sign
      };

      controlPoint2 = {
        x: midX,
        y: targetPoint.y - vertOffset * sign
      };
    } else if (mainDirection === 'vertical') {
      // 垂直方向的曲线控制点
      const dy = targetPoint.y - sourcePoint.y;
      const midY = sourcePoint.y + dy * 0.5;

      // 根据曲率计算控制点的水平偏移
      const horizOffset = Math.min(Math.abs(dy) * 0.3, 80) * curvature;
      const sign = sourcePoint.x > targetPoint.x ? -1 : 1; // 确定偏移方向

      controlPoint1 = {
        x: sourcePoint.x + horizOffset * sign,
        y: midY
      };

      controlPoint2 = {
        x: targetPoint.x - horizOffset * sign,
        y: midY
      };
    } else {
      // 对角线方向的控制点，使用出入口方向
      const sourceDir = getNodeExitDirection(edge.sourceId, targetCenter);
      const targetDir = getNodeEntryDirection(edge.targetId, source);

      controlPoint1 = calculateControlPoint(sourcePoint, sourceDir, curvature);
      controlPoint2 = calculateControlPoint(targetPoint, targetDir, curvature);
    }

    // 返回三次贝塞尔曲线路径
    return `M ${sourcePoint.x} ${sourcePoint.y} C ${controlPoint1.x} ${controlPoint1.y}, ${controlPoint2.x} ${controlPoint2.y}, ${targetPoint.x} ${targetPoint.y}`;
  }
};

// 检查两个节点是否在同一水平线或垂直线上
const checkNodesAlignment = (point1: {x: number, y: number}, point2: {x: number, y: number}) => {
  // 使用配置的容差范围
  const tolerance = props.alignmentTolerance;

  // 检查是否接近同一水平线
  const isHorizontallyAligned = Math.abs(point1.y - point2.y) <= tolerance;

  // 检查是否接近同一垂直线
  const isVerticallyAligned = Math.abs(point1.x - point2.x) <= tolerance;

  // 如果节点出入口点在同一水平线或垂直线上，则返回true
  return isHorizontallyAligned || isVerticallyAligned;
};

// 确定节点的出口方向（从节点出发的方向）
const getNodeExitDirection = (nodeId: number, targetPoint: {x: number, y: number}) => {
  const node = nodes.value.find(n => n.id.value === nodeId);
  if (!node) return 'right';

  const center = {
    x: node.position.x + node.shape.width / 2,
    y: node.position.y + node.shape.height / 2
  };

  // 计算角度
  const angle = calculateAngle(center.x, center.y, targetPoint.x, targetPoint.y);

  // 根据节点到目标的角度确定出口方向
  if (angle >= 315 || angle < 45) return 'right';
  if (angle >= 45 && angle < 135) return 'bottom';
  if (angle >= 135 && angle < 225) return 'left';
  return 'top';
};

// 确定节点的入口方向（进入节点的方向）
const getNodeEntryDirection = (nodeId: number, sourcePoint: {x: number, y: number}) => {
  const node = nodes.value.find(n => n.id.value === nodeId);
  if (!node) return 'left';

  const center = {
    x: node.position.x + node.shape.width / 2,
    y: node.position.y + node.shape.height / 2
  };

  // 计算角度
  const angle = calculateAngle(sourcePoint.x, sourcePoint.y, center.x, center.y);

  // 根据源点到节点的角度确定入口方向
  if (angle >= 315 || angle < 45) return 'left';
  if (angle >= 45 && angle < 135) return 'top';
  if (angle >= 135 && angle < 225) return 'right';
  return 'bottom';
};

// 根据位置和方向计算控制点
const calculateControlPoint = (point: {x: number, y: number}, direction: string, curvature: number) => {
  // 控制点应该沿着指定方向延伸
  const baseDistance = 150; // 基础控制距离
  const controlDistance = baseDistance * curvature; // 根据曲率调整控制点距离

  const controlPoint = { x: point.x, y: point.y };

  switch (direction) {
    case 'right':
      controlPoint.x += controlDistance;
      break;
    case 'left':
      controlPoint.x -= controlDistance;
      break;
    case 'bottom':
      controlPoint.y += controlDistance;
      break;
    case 'top':
      controlPoint.y -= controlDistance;
      break;
  }

  return controlPoint;
};
</script>

<style scoped>
.edge-layer {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 0;
}

.edge-path {
  transition: stroke 0.3s ease, stroke-width 0.3s ease;
  pointer-events: all;
  cursor: pointer;
}

.edge-path.hovered {
  stroke-width: 3px;
  filter: drop-shadow(0 0 3px rgba(0, 0, 0, 0.3));
}

.edge-label {
  font-size: 12px;
  fill: #64748B;
  pointer-events: none;
  transition: font-weight 0.3s ease, fill 0.3s ease;
}

.edge-label.hovered {
  font-weight: bold;
  fill: #475569;
}

.edge-tooltip {
  background-color: white;
  border-radius: 4px;
  padding: 8px 12px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
  position: relative;
  font-size: 12px;
  color: #334155;
  white-space: nowrap;
  user-select: none;
}

.close-btn {
  position: absolute;
  top: 4px;
  right: 8px;
  cursor: pointer;
  font-size: 16px;
  color: #94A3B8;
}

.close-btn:hover {
  color: #475569;
}
</style>
