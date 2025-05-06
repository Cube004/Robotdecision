<template>
  <div class="add-node-container" v-if="visible">
    <!-- 顶部提示栏 -->
    <div class="tip-bar" :class="{ 'confirmation-active': showConfirmation }">
      <div v-if="!showConfirmation" class="tip-content">
        <div class="tip-icon">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <rect x="2.9" y="4.9" width="18.2" height="14.2" rx="3.1" stroke="currentColor" stroke-width="1.8"></rect>
          </svg>
        </div>
        <div class="tip-text" v-if="step === 1">
          是否添加新节点？
        </div>
        <div class="tip-text" v-else-if="step === 2">
          点击画布放置节点
        </div>
      </div>

      <!-- 确认添加提示 -->
      <div v-if="showConfirmation" class="confirmation-prompt">
        <span>是否添加新节点？</span>
        <div class="confirmation-actions">
          <button class="confirm-btn" @click="($event) => confirmAddNode($event)">确认</button>
          <button class="cancel-btn" @click="($event) => cancelAddNode($event)">取消</button>
        </div>
      </div>
    </div>

    <!-- 跟随鼠标的节点预览 -->
    <div
      v-if="isPlacingNode"
      class="node-preview"
      :style="{
        left: `${mousePosition.x}px`,
        top: `${mousePosition.y}px`
      }"
    >
      <div class="node-icon">
        <svg class="icon-svg" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
          <path d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2m-6 9l2 2 4-4" fill="currentColor" />
        </svg>
      </div>
      <div class="node-content">任务节点</div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, onMounted, onUnmounted, defineProps } from 'vue';
import { handleConfirmAddNode, handleCancelAddNode, handlePlaceNode } from '@/types/extensions/ToolMenu/AddNode';

const props = defineProps<{
  visible: boolean;
}>();

const step = ref(1); // 1: 询问是否添加, 2: 放置节点
const showConfirmation = ref(true); // 默认显示确认提示
const isPlacingNode = ref(false); // 是否正在放置节点
const mousePosition = ref({ x: 0, y: 0 }); // 鼠标位置
const clickDelay = ref(false); // 点击延迟标志

// 监听visible属性变化
watch(() => props.visible, (newVal) => {
  if (newVal) {
    // 重置状态
    step.value = 1;
    showConfirmation.value = true;
    isPlacingNode.value = false;
  }
});

// 确认添加节点
const confirmAddNode = (event: MouseEvent) => {
  // 阻止事件冒泡
  event.stopPropagation();

  showConfirmation.value = false;
  step.value = 2;
  isPlacingNode.value = true;

  // 设置点击延迟标志，防止误触
  clickDelay.value = true;
  setTimeout(() => {
    clickDelay.value = false;
  }, 300); // 300毫秒延迟
  handleConfirmAddNode();
};

// 取消添加节点
const cancelAddNode = (event: MouseEvent|null) => {
  // 阻止事件冒泡
  event?.stopPropagation();

  // 设置点击延迟标志，防止误触
  clickDelay.value = true;
  setTimeout(() => {
    clickDelay.value = false;
  }, 300); // 300毫秒延迟

  resetState();
  handleCancelAddNode();
};

// 处理鼠标移动
const handleMouseMove = (event: MouseEvent) => {
  if (isPlacingNode.value) {
    mousePosition.value = {
      x: event.clientX,
      y: event.clientY
    };
  }
};

// 处理鼠标点击
const handleMouseClick = (event: MouseEvent) => {
  // 检查点击事件是否来自确认或取消按钮
  const target = event.target as HTMLElement;
  if (target.closest('.confirm-btn') || target.closest('.cancel-btn') || clickDelay.value) {
    return; // 如果点击的是按钮或在延迟期间，则忽略
  }

  if (isPlacingNode.value) {
    handlePlaceNode({
      x: event.clientX,
      y: event.clientY
    });
    resetState();
  }
};

// 重置状态
const resetState = () => {
  step.value = 1;
  showConfirmation.value = true;
  isPlacingNode.value = false;
};

// 监听鼠标事件
onMounted(() => {
  window.addEventListener('mousemove', handleMouseMove);
  window.addEventListener('click', handleMouseClick);
});

onUnmounted(() => {
  window.removeEventListener('mousemove', handleMouseMove);
  window.removeEventListener('click', handleMouseClick);
});
</script>

<style scoped>
.add-node-container {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none; /* 允许点击穿透 */
  z-index: 1001;
}

.tip-bar {
  background-color: #fff;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  border-radius: 0 0 8px 8px;
  padding: 12px 20px;
  margin: 0 auto;
  width: max-content;
  min-width: 300px;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.3s ease;
  transform: translateY(0);
  pointer-events: auto; /* 允许点击 */
}

.tip-bar.confirmation-active {
  transform: translateY(0);
}

.tip-content {
  display: flex;
  align-items: center;
  gap: 10px;
}

.tip-icon {
  color: #3B82F6;
  display: flex;
  align-items: center;
  justify-content: center;
}

.tip-text {
  font-size: 14px;
  color: #334155;
  font-weight: 500;
}

.confirmation-prompt {
  display: flex;
  align-items: center;
  justify-content: space-between;
  width: 100%;
}

.confirmation-prompt span {
  font-size: 14px;
  color: #334155;
  font-weight: 500;
}

.confirmation-actions {
  display: flex;
  gap: 10px;
}

.confirm-btn, .cancel-btn {
  padding: 6px 12px;
  border-radius: 4px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  border: none;
  transition: all 0.2s ease;
}

.confirm-btn {
  background-color: #3B82F6;
  color: white;
}

.confirm-btn:hover {
  background-color: #2563EB;
}

.cancel-btn {
  background-color: #F1F5F9;
  color: #64748B;
}

.cancel-btn:hover {
  background-color: #E2E8F0;
}

.node-preview {
  position: fixed;
  display: flex;
  align-items: center;
  width: 200px;
  height: 60px;
  background-color: white;
  border: 1px solid #E2E8F0;
  border-radius: 8px;
  box-shadow: 0 2px 12px rgba(0, 0, 0, 0.1);
  transform: translate(-50%, -50%);
  pointer-events: none; /* 防止节点预览阻挡点击 */
  z-index: 1000;
}

.node-icon {
  height: 100%;
  width: 40px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #3B82F6;
  color: white;
  border-top-left-radius: 8px;
  border-bottom-left-radius: 8px;
}

.icon-svg {
  width: 20px;
  height: 20px;
}

.node-content {
  flex: 1;
  padding: 0 16px;
  font-size: 14px;
  color: #334155;
  font-weight: 500;
}
</style>
