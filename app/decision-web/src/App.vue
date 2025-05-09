<template>
  <div>
    <div class="viewport-container" ref="viewportContainer" @mousemove="updateMouseCanvasPosition">
      <div
        class="scale-content"
        ref="scaleContent"
        :style="{
          transform: `translate(${position.x}px, ${position.y}px) scale(${currentScale})`,
          transformOrigin: '0 0',
          cursor: isCtrlPressed ? 'grab' : 'default'
        }"
        @mousedown="startDrag"
        @mousemove="onDrag"
        @mouseup="stopDrag"
        @mouseleave="stopDrag"
      >
        <canvas ref="backgroundLayer"></canvas>
        <!-- 对象元素 -->
        <EdgeComponent
        :arrow-offset="0"
        :alignment-tolerance="15"
        @edge-click="handleEdgeClick"
        v-if="layers.find(layer => layer.name === '连接线图层')?.visible"
        />
        <NodeGroupItem
        v-if="layers.find(layer => layer.name === '编组图层')?.visible"
        />
        <NodeItem @open-edit-menu="handleNodeClick"
        v-if="layers.find(layer => layer.name === '节点图层')?.visible"
        />
      </div>
    </div>

    <!-- 可选：坐标显示 -->
    <div class="coordinates-display" v-if="showCoordinates">
      <span>X: {{ Math.round(canvasX) }}</span>
      <span>Y: {{ Math.round(canvasY) }}</span>
    </div>

    <!-- 缩放控制器 -->
    <div class="zoom-controls">
      <button class="zoom-button" @click="zoomOut" title="缩小">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M5 12H19" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
      </button>
      <div class="zoom-value">{{ Math.round(currentScale * 100) }}%</div>
      <button class="zoom-button" @click="zoomIn" title="放大">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M12 5V19" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          <path d="M5 12H19" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
      </button>
      <button class="reset-button" @click="resetZoom" title="重置">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M3 12C3 16.9706 7.02944 21 12 21C16.9706 21 21 16.9706 21 12C21 7.02944 16.9706 3 12 3" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          <path d="M3 4L3 8L7 8" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          <path d="M3 8L8 3" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
        <span>重置</span>
      </button>
      <!-- 可选：添加显示/隐藏坐标的切换按钮 -->
      <button class="coordinate-toggle" @click="showCoordinates = !showCoordinates" title="显示/隐藏坐标">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M3 3V21H21" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          <circle cx="16" cy="8" r="3" stroke="currentColor" stroke-width="2" />
        </svg>
      </button>
    </div>

    <!-- 拖拽提示 -->
    <div class="drag-hint" v-if="activeToolId === 'cursor' && isCtrlPressed">
      <span>按住Ctrl键并拖动鼠标可移动画布</span>
    </div>

    <!-- 工具菜单 -->
    <ToolMenu />

    <!-- 编辑菜单 -->
    <EditMenu
      :visible="editMenuVisible"
      :selected-node-id="selectedNodeId"
      @update:visible="editMenuVisible = $event"
      @node-updated="handleNodeUpdated"
    />
    <EdgeMenu
      :visible="edgeMenuVisible"
      :selected-edge-id="selectedEdgeId"
      @update:visible="edgeMenuVisible = $event"
      @edge-updated="handleEdgeUpdated"
      @close="edgeMenuVisible = false"
    />

    <!-- 错误弹窗 -->
    <ErrorDialog
      :visible="errorDialogVisible"
      :message="errorMessage"
      @update:visible="errorDialogVisible = $event"
      @close="clearError"
    />

    <!-- 使用确认窗口组件 -->
    <ConfirmWindow
      v-model:visible="confirmVisible"
      :message="confirmMessage"
      :title="confirmTitle"
      @confirm="handleConfirmResult"
      ref="confirmWindowRef"
      />

    <!-- 任务编组面板 -->
    <NodeGroupPanel
      :is-open="activeToolId === 'renderNodeGroup'"
      @close="activeToolId = 'cursor'"
      @node-group-click="handleNodeGroupClick"
    />
  </div>
</template>

<script setup lang="ts" name="App">
  import NodeItem from '@/components/NodeItem.vue';
  import EdgeComponent from '@/components/EdgeComponent.vue';
  import ToolMenu from '@/components/ToolMenu.vue';
  import EditMenu from '@/components/NodeMenu.vue';
  import ErrorDialog from '@/components/ErrorWindow.vue';
  import { onMounted, ref, onUnmounted, computed } from 'vue';
  import { example } from '@/types/example';
  import EdgeMenu from '@/components/EdgeMenu.vue';
  import type { Edge } from '@/types/EdgeBase';
  import NodeGroupItem from '@/components/NodeGroupItem.vue';
  import { activeToolId, mousePositionCanvas, currentScale } from './types/Manger';
  import NodeGroupPanel from '@/components/NodeGroupPanel.vue';
  import { confirmVisible, confirmMessage, confirmTitle, handleConfirmResult ,confirmWindowRef} from '@/types/ConfirmDialog';
  import ConfirmWindow from '@/components/ConfirmWindow.vue';
  import { layers } from '@/types/Layers';
  // 是否显示坐标信息
  const showCoordinates = ref(true);

  // 计算画布坐标X和Y（用于显示）
  const canvasX = computed(() => Math.round(mousePositionCanvas.value.x));
  const canvasY = computed(() => Math.round(mousePositionCanvas.value.y));

  // 缩放系统
  const viewportContainer = ref<HTMLElement | null>(null);
  const scaleContent = ref<HTMLElement | null>(null);
  const minScale = 0.4;
  const maxScale = 2.0;
  const scaleStep = 0.1;

  // 平移系统
  const position = ref({ x: 0, y: 0 });
  const isDragging = ref(false);
  const dragStart = ref({ x: 0, y: 0 });
  const dragStartPosition = ref({ x: 0, y: 0 });
  const isCtrlPressed = ref(false);

  // 监听Ctrl键
  const handleKeyDown = (e: KeyboardEvent) => {
    if (e.key === 'Control') {
      isCtrlPressed.value = true;
      if (scaleContent.value && activeToolId.value === 'cursor') {
        scaleContent.value.style.cursor = 'grab';
      }
    }
  };

  const handleKeyUp = (e: KeyboardEvent) => {
    if (e.key === 'Control') {
      isCtrlPressed.value = false;
      if (scaleContent.value) {
        scaleContent.value.style.cursor = 'default';
      }
      // 如果正在拖拽，停止拖拽
      if (isDragging.value) {
        stopDrag();
      }
    }
  };

  // 当窗口失去焦点时，确保释放Ctrl键状态
  const handleWindowBlur = () => {
    isCtrlPressed.value = false;
    if (isDragging.value) {
      stopDrag();
    }
  };

  // 拖拽功能
  const startDrag = (e: MouseEvent) => {
    // 只有在使用光标工具且按下Ctrl键时才允许拖拽
    if (activeToolId.value !== 'cursor' || !e.ctrlKey) return;

    isDragging.value = true;
    dragStart.value = { x: e.clientX, y: e.clientY };
    dragStartPosition.value = { x: position.value.x, y: position.value.y };

    // 改变鼠标样式
    if (scaleContent.value) {
      scaleContent.value.style.cursor = 'grabbing';
    }
  };

  const onDrag = (e: MouseEvent) => {
    if (!isDragging.value) return;

    const dx = e.clientX - dragStart.value.x;
    const dy = e.clientY - dragStart.value.y;

    // 计算新位置
    let newX = dragStartPosition.value.x + dx;
    let newY = dragStartPosition.value.y + dy;

    // 应用边界限制
    if (viewportContainer.value && backgroundLayer.value) {
      const viewportWidth = viewportContainer.value.clientWidth;
      const viewportHeight = viewportContainer.value.clientHeight;
      const canvasWidth = backgroundLayer.value.width * currentScale.value;
      const canvasHeight = backgroundLayer.value.height * currentScale.value;

      // 计算边界限制
      // 左边界: 确保右侧不会出现空白
      const minX = Math.min(viewportWidth - canvasWidth, 0);
      // 上边界: 确保底部不会出现空白
      const minY = Math.min(viewportHeight - canvasHeight, 0);

      // 应用限制
      newX = Math.max(minX, Math.min(0, newX));
      newY = Math.max(minY, Math.min(0, newY));
    }

    position.value = { x: newX, y: newY };
  };

  const stopDrag = () => {
    isDragging.value = false;

    // 恢复鼠标样式
    if (scaleContent.value) {
      scaleContent.value.style.cursor = isCtrlPressed.value ? 'grab' : 'default';
    }
  };

  // 监听缩放事件，实现以鼠标位置为中心点的缩放
  const handleWheelZoom = (e: WheelEvent) => {
    // 检查是否在可缩放容器内
    if (viewportContainer.value && viewportContainer.value.contains(e.target as Node)) {
      e.preventDefault();

      // 获取鼠标相对于viewport的位置
      const rect = viewportContainer.value.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      // 计算鼠标相对于画布内容的位置（考虑当前平移和缩放）
      const contentX = (mouseX - position.value.x) / currentScale.value;
      const contentY = (mouseY - position.value.y) / currentScale.value;

      // 决定缩放方向
      const direction = e.deltaY < 0 ? 1 : -1;
      const factor = 1 + scaleStep * direction;

      // 限制缩放范围
      const newScale = Math.max(minScale, Math.min(maxScale, currentScale.value * factor));

      // 只有在范围内才执行缩放
      if (newScale !== currentScale.value) {
        // 更新缩放值
        currentScale.value = newScale;

        // 调整位置以保持鼠标下方的内容不变
        position.value = {
          x: mouseX - contentX * newScale,
          y: mouseY - contentY * newScale
        };

        // 应用边界限制，使用requestAnimationFrame确保在渲染前应用
        requestAnimationFrame(() => {
          applyBoundaryLimits();
        });
      }
    }
  };

  // 应用边界限制的函数，可以在缩放后调用以确保位置有效
  const applyBoundaryLimits = () => {
    if (viewportContainer.value && backgroundLayer.value) {
      const viewportWidth = viewportContainer.value.clientWidth;
      const viewportHeight = viewportContainer.value.clientHeight;
      const canvasWidth = backgroundLayer.value.width * currentScale.value;
      const canvasHeight = backgroundLayer.value.height * currentScale.value;

      // 如果画布小于视口，居中显示
      if (canvasWidth <= viewportWidth) {
        position.value.x = (viewportWidth - canvasWidth) / 2;
      } else {
        // 否则确保不会出现空白区域
        const minX = Math.min(viewportWidth - canvasWidth, 0);
        position.value.x = Math.max(minX, Math.min(0, position.value.x));
      }

      if (canvasHeight <= viewportHeight) {
        position.value.y = (viewportHeight - canvasHeight) / 2;
      } else {
        const minY = Math.min(viewportHeight - canvasHeight, 0);
        position.value.y = Math.max(minY, Math.min(0, position.value.y));
      }
    }
  };

  // 缩放功能
  const zoomIn = () => {
    if (currentScale.value < maxScale) {
      currentScale.value = Math.min(currentScale.value + scaleStep, maxScale);
      applyBoundaryLimits();
    }
  };

  const zoomOut = () => {
    if (currentScale.value > minScale) {
      currentScale.value = Math.max(currentScale.value - scaleStep, minScale);
      applyBoundaryLimits();
    }
  };

  const resetZoom = () => {
    currentScale.value = 1.0;
    // 重新计算初始位置，使画布居中
    if (viewportContainer.value && backgroundLayer.value) {
      const viewportWidth = viewportContainer.value.clientWidth;
      const viewportHeight = viewportContainer.value.clientHeight;
      const canvasWidth = backgroundLayer.value.width;
      const canvasHeight = backgroundLayer.value.height;

      position.value = {
        x: (viewportWidth - canvasWidth) / 2,
        y: (viewportHeight - canvasHeight) / 2
      };
    }
  };

  // 计算鼠标在画布中的实际坐标（考虑缩放和平移）
  const getCanvasCoordinates = (clientX: number, clientY: number) => {
    // 确保视口容器已挂载
    if (!viewportContainer.value) return { x: 0, y: 0 };

    // 获取视口容器的边界矩形
    const rect = viewportContainer.value.getBoundingClientRect();

    // 计算鼠标相对于视口容器的位置
    const viewportX = clientX - rect.left;
    const viewportY = clientY - rect.top;

    // 考虑平移和缩放，计算实际画布坐标
    const canvasX = (viewportX - position.value.x) / currentScale.value;
    const canvasY = (viewportY - position.value.y) / currentScale.value;

    return { x: canvasX, y: canvasY };
  };

  // 更新鼠标在画布中的坐标
  const updateMouseCanvasPosition = (e: MouseEvent) => {
    const canvasCoords = getCanvasCoordinates(e.clientX, e.clientY);
    mousePositionCanvas.value = canvasCoords;
  };

  // 定义节点和边的响应式数据
  const backgroundLayer = ref<HTMLCanvasElement | null>(null);
  const editMenuVisible = ref(false);
  const selectedNodeId = ref<number | null>(null);

  // EdgeMenu相关状态
  const edgeMenuVisible = ref(false);
  const selectedEdgeId = ref<number | null>(null);

  // 错误弹窗相关状态
  const errorDialogVisible = ref(false);
  const errorMessage = ref('');

  // 处理节点点击事件
  const handleNodeClick = (nodeId: number) => {
    selectedNodeId.value = nodeId;
    editMenuVisible.value = true;
    activeToolId.value = 'cursor';
    // 关闭边编辑菜单
    edgeMenuVisible.value = false;
  };

  // 处理边点击事件
  const handleEdgeClick = (edgeId: number) => {
    selectedEdgeId.value = edgeId;
    edgeMenuVisible.value = true;
    activeToolId.value = 'cursor';
    // 关闭节点编辑菜单
    editMenuVisible.value = false;
  };

  // 处理任务编组点击事件
  const handleNodeGroupClick = () => {
    // 关闭边编辑菜单
    edgeMenuVisible.value = false;
    // 关闭节点编辑菜单
    editMenuVisible.value = false;
  };

  // 处理节点更新事件
  const handleNodeUpdated = (node: Node & { type?: string; mode?: string; resetTime?: number; waypointId?: string }) => {
    console.log('节点已更新', node);
    // 这里可以添加其他逻辑，例如保存到服务器
  };

  // 处理边更新事件
  const handleEdgeUpdated = (edge: Edge) => {
    console.log('连线已更新', edge);
    // 这里可以添加其他逻辑，例如保存到服务器
  };

  // 显示错误信息
  const showError = (message: string) => {
    errorMessage.value = message;
    errorDialogVisible.value = true;
  };

  // 清除错误信息
  const clearError = () => {
    errorMessage.value = '';
  };

  onMounted(() => {
    // 添加滚轮缩放事件监听
    window.addEventListener('wheel', handleWheelZoom, { passive: false });

    // 添加键盘事件监听
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    window.addEventListener('blur', handleWindowBlur);

    // 添加鼠标移动事件监听，以跟踪画布坐标
    window.addEventListener('mousemove', updateMouseCanvasPosition);

    // 绘制背景网格
    const ctx = backgroundLayer.value?.getContext('2d');
    if (backgroundLayer.value && ctx){
      // 设置更大的画布尺寸，实现无限平面的感觉
      const canvasWidth = window.innerWidth * 3;
      const canvasHeight = window.innerHeight * 3;

      backgroundLayer.value.width = canvasWidth;
      backgroundLayer.value.height = canvasHeight;

      ctx.fillStyle = '#F2F4F7';
      ctx.fillRect(0, 0, canvasWidth, canvasHeight);
      const gridInterval = 20; // 固定网格间距

      // 计算最大有效索引
      const maxXIndex = Math.floor(canvasWidth / gridInterval);
      const maxYIndex = Math.floor(canvasHeight / gridInterval);

      for (let xIndex = 0; xIndex <= maxXIndex; xIndex++) {
          for (let yIndex = 0; yIndex <= maxYIndex; yIndex++) {
              const x = xIndex * gridInterval;
              const y = yIndex * gridInterval;

              ctx.beginPath();
              ctx.fillStyle = '#E2E4EC';
              ctx.arc(x, y, 0.9, 0, Math.PI * 2);
              ctx.fill();
          }
      }
    }

    // 居中画布
    if (viewportContainer.value && backgroundLayer.value) {
      const viewportWidth = viewportContainer.value.clientWidth;
      const viewportHeight = viewportContainer.value.clientHeight;
      const canvasWidth = backgroundLayer.value.width;
      const canvasHeight = backgroundLayer.value.height;

      // 将画布内容居中显示
      position.value = {
        x: (viewportWidth - canvasWidth) / 2,
        y: (viewportHeight - canvasHeight) / 2
      };
      const init_position = {
        x: Math.abs((viewportWidth - canvasWidth) / 2),
        y: Math.abs((viewportHeight - canvasHeight) / 2)
      }
      example(init_position);
    }
    // 示例：监听全局错误事件
    window.addEventListener('error', (event) => {
      showError(`发生错误: ${event.message}`);
    });

    // 示例：监听自定义错误事件
    window.addEventListener('custom-error', ((event: CustomEvent) => {
      showError(event.detail.message || '发生未知错误');
    }) as EventListener);
  });

  // 在组件卸载时移除事件监听器
  onUnmounted(() => {
    window.removeEventListener('wheel', handleWheelZoom);
    window.removeEventListener('keydown', handleKeyDown);
    window.removeEventListener('keyup', handleKeyUp);
    window.removeEventListener('blur', handleWindowBlur);
    window.removeEventListener('mousemove', updateMouseCanvasPosition);
  });

  // 导出方法，供其他组件使用
  defineExpose({
    showError,
    getCanvasCoordinates // 公开坐标转换函数
  });
</script>


<style scoped>
  canvas {
    position: absolute;
    top: 0;
    left: 0;
    /* 修改为更大的尺寸 */
    width: 300%;
    height: 300%;
  }

  .viewport-container {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    overflow: hidden;
    z-index: 0;
  }

  .scale-content {
    position: absolute;
    /* 尺寸设置为足够大 */
    width: 300%;
    height: 300%;
    will-change: transform; /* 优化性能 */
    touch-action: none; /* 防止移动设备上的默认滚动行为 */
  }

  .coordinates-display {
    position: fixed;
    top: 20px;
    left: 20px;
    background-color: rgba(255, 255, 255, 0.9);
    padding: 8px 12px;
    border-radius: 6px;
    box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
    font-family: 'Courier New', monospace;
    font-size: 14px;
    display: flex;
    gap: 12px;
    color: #334155;
    z-index: 1000;
    border: 1px solid #e2e8f0;
    backdrop-filter: blur(4px);
  }

  .coordinates-display span {
    min-width: 60px;
  }

  .drag-hint {
    position: fixed;
    top: 20px;
    left: 50%;
    transform: translateX(-50%);
    background-color: rgba(0, 0, 0, 0.7);
    color: white;
    padding: 8px 16px;
    border-radius: 20px;
    font-size: 14px;
    z-index: 1000;
    pointer-events: none;
    animation: fadeIn 0.3s ease-in-out;
  }

  @keyframes fadeIn {
    from { opacity: 0; transform: translateX(-50%) translateY(-10px); }
    to { opacity: 1; transform: translateX(-50%) translateY(0); }
  }

  .zoom-controls {
    position: fixed;
    bottom: 20px;
    right: 20px;
    display: flex;
    align-items: center;
    background-color: white;
    border-radius: 30px;
    box-shadow: 0 3px 15px rgba(0, 0, 0, 0.15);
    padding: 8px 12px;
    z-index: 1000;
    backdrop-filter: blur(10px);
    border: 1px solid rgba(234, 234, 234, 0.8);
    transition: all 0.2s ease;
  }

  .zoom-controls:hover {
    box-shadow: 0 5px 20px rgba(0, 0, 0, 0.2);
  }

  .zoom-button,
  .coordinate-toggle {
    background-color: transparent;
    border: none;
    width: 32px;
    height: 32px;
    border-radius: 50%;
    margin: 0 2px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    color: #506380;
    transition: all 0.15s ease;
  }

  .zoom-button:hover,
  .coordinate-toggle:hover {
    background-color: #f0f4f8;
    color: #334155;
  }

  .zoom-button:active,
  .coordinate-toggle:active {
    background-color: #e6ebf1;
    transform: scale(0.95);
  }

  .coordinate-toggle {
    margin-left: 8px;
    border-left: 1px solid #eaeaea;
    padding-left: 8px;
  }

  .zoom-value {
    margin: 0 10px;
    font-size: 14px;
    font-weight: 500;
    min-width: 45px;
    text-align: center;
    color: #334155;
    font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
  }

  .reset-button {
    background-color: #f0f4f8;
    border: none;
    height: 32px;
    border-radius: 16px;
    margin-left: 8px;
    padding: 0 12px;
    cursor: pointer;
    display: flex;
    align-items: center;
    gap: 6px;
    color: #506380;
    transition: all 0.15s ease;
    font-size: 13px;
    font-weight: 500;
  }

  .reset-button:hover {
    background-color: #e6ebf1;
    color: #334155;
  }

  .reset-button:active {
    transform: scale(0.98);
  }

  .control-panel {
    position: fixed;
    top: 20px;
    right: 20px;
    background-color: white;
    padding: 15px;
    border-radius: 8px;
    box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
    z-index: 100;
    min-width: 250px;
  }

  .control-item {
    margin-bottom: 10px;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  .control-item.checkbox {
    margin-top: 15px;
  }

  .control-item.checkbox label {
    display: flex;
    align-items: center;
    gap: 5px;
    cursor: pointer;
  }

  .control-item label {
    font-size: 14px;
    color: #334155;
    margin-right: 10px;
  }

  .control-item input[type="range"] {
    width: 120px;
    margin: 0 10px;
  }

  .control-item span {
    font-size: 14px;
    color: #64748B;
    min-width: 40px;
    text-align: right;
  }
</style>
