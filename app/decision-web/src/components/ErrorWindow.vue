<template>
  <div class="error-overlay" v-if="visible" @click.self="close">
    <div class="error-dialog" :class="{ 'show': animationState }">
      <!-- 顶部导航栏和标题 -->
      <div class="error-header">
        <div class="error-icon">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M12 8V12" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M12 16H12.01" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
        </div>
        <h3>错误提示</h3>
        <button class="close-button" @click="close" title="关闭">×</button>
      </div>

      <!-- 错误信息区域 -->
      <div class="error-content">
        <div class="error-message">
          {{ message }}
        </div>
      </div>

      <!-- 底部按钮区域 -->
      <div class="error-footer">
        <button class="primary-button" @click="close">确认</button>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, nextTick } from 'vue';

// 定义组件属性
const props = defineProps<{
  visible: boolean;
  message: string;
}>();

// 定义事件
const emit = defineEmits<{
  (e: 'update:visible', value: boolean): void;
  (e: 'close'): void;
}>();

// 用于动画效果的状态
const animationState = ref(false);

// 监听visible属性的变化，添加动画效果
watch(() => props.visible, async (newValue) => {
  if (newValue) {
    await nextTick();
    animationState.value = true;
  } else {
    animationState.value = false;
  }
}, { immediate: true });

// 关闭弹窗方法
const close = () => {
  animationState.value = false;
  setTimeout(() => {
    emit('update:visible', false);
    emit('close');
  }, 300); // 等待动画完成
};
</script>

<style scoped>
.error-overlay {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 2000;
  backdrop-filter: blur(2px);
}

.error-dialog {
  background-color: white;
  border-radius: 8px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  width: 400px;
  max-width: 90%;
  opacity: 0;
  transform: scale(0.9);
  transition: all 0.3s ease;
  overflow: hidden;
}

.error-dialog.show {
  opacity: 1;
  transform: scale(1);
}

.error-header {
  display: flex;
  align-items: center;
  padding: 16px 20px;
  border-bottom: 1px solid #f0f0f0;
  background-color: #f9f9f9;
}

.error-icon {
  display: flex;
  align-items: center;
  margin-right: 12px;
  color: #e53935;
}

.error-header h3 {
  margin: 0;
  flex: 1;
  font-size: 16px;
  font-weight: 500;
  color: #333;
}

.close-button {
  background: none;
  border: none;
  font-size: 22px;
  line-height: 1;
  color: #666;
  cursor: pointer;
  padding: 0;
  width: 24px;
  height: 24px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 4px;
  transition: background-color 0.2s;
}

.close-button:hover {
  background-color: rgba(0, 0, 0, 0.05);
}

.error-content {
  padding: 20px;
}

.error-message {
  color: #333;
  line-height: 1.5;
  font-size: 14px;
  word-break: break-word;
}

.error-footer {
  padding: 12px 20px;
  display: flex;
  justify-content: flex-end;
  border-top: 1px solid #f0f0f0;
}

.primary-button {
  background-color: #2196f3;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 8px 16px;
  font-size: 14px;
  cursor: pointer;
  transition: background-color 0.2s;
}

.primary-button:hover {
  background-color: #1976d2;
}
</style>
