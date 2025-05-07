<template>
  <div class="confirm-overlay" v-if="visible" @click.self="handleCancel">
    <div class="confirm-dialog" :class="{ 'show': animationState }">
      <!-- 顶部导航栏和标题 -->
      <div class="confirm-header">
        <div class="confirm-icon">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M12 8V12" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            <path d="M12 16H12.01" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
        </div>
        <h3>{{ title }}</h3>
        <button class="close-button" @click="handleCancel" title="关闭">×</button>
      </div>

      <!-- 确认信息区域 -->
      <div class="confirm-content">
        <div class="confirm-message">
          {{ message }}
        </div>
      </div>

      <!-- 底部按钮区域 -->
      <div class="confirm-footer">
        <button class="cancel-button" @click="handleCancel">取消</button>
        <button class="confirm-button" @click="handleConfirm">确认</button>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, nextTick } from 'vue';

// 定义组件属性
const props = withDefaults(defineProps<{
  visible: boolean;
  message: string;
  title?: string;
}>(), {
  title: '确认提示'
});

// 定义事件
const emit = defineEmits<{
  (e: 'update:visible', value: boolean): void;
  (e: 'confirm', value: boolean): void;
}>();

// 用于动画效果的状态
const animationState = ref(false);

// 存储resolve函数的引用
let resolvePromise: ((value: boolean) => void) | null = null;

// 监听visible属性的变化，添加动画效果
watch(() => props.visible, async (newValue) => {
  if (newValue) {
    await nextTick();
    animationState.value = true;
  } else {
    animationState.value = false;
  }
}, { immediate: true });


// 创建Promise，提供给外部调用
const showConfirm = (): Promise<boolean> => {
  // 使用自定义事件通知父组件更新消息和标题
  emit('update:visible', true);
  return new Promise<boolean>((resolve) => {
    resolvePromise = resolve;
  });
};

// 确认按钮处理
const handleConfirm = () => {
  closeWithResult(true);
};

// 取消按钮处理
const handleCancel = () => {
  closeWithResult(false);
};

// 关闭弹窗并返回结果
const closeWithResult = (result: boolean) => {
  animationState.value = false;

  setTimeout(() => {
    emit('update:visible', false);
    emit('confirm', result);

    // 如果有等待的Promise，则resolve它
    if (resolvePromise) {
      resolvePromise(result);
      resolvePromise = null;
    }
  }, 20); // 等待动画完成
};

// 对外暴露方法
defineExpose({
  showConfirm
});
</script>

<style scoped>
.confirm-overlay {
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

.confirm-dialog {
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

.confirm-dialog.show {
  opacity: 1;
  transform: scale(1);
}

.confirm-header {
  display: flex;
  align-items: center;
  padding: 16px 20px;
  border-bottom: 1px solid #f0f0f0;
  background-color: #f9f9f9;
}

.confirm-icon {
  display: flex;
  align-items: center;
  margin-right: 12px;
  color: #2196f3; /* 使用蓝色表示确认 */
}

.confirm-header h3 {
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

.confirm-content {
  padding: 20px;
}

.confirm-message {
  color: #333;
  line-height: 1.5;
  font-size: 14px;
  word-break: break-word;
}

.confirm-footer {
  padding: 12px 20px;
  display: flex;
  justify-content: flex-end;
  border-top: 1px solid #f0f0f0;
  gap: 10px;
}

.cancel-button {
  background-color: #f5f5f5;
  color: #333;
  border: 1px solid #ddd;
  border-radius: 4px;
  padding: 8px 16px;
  font-size: 14px;
  cursor: pointer;
  transition: background-color 0.2s;
}

.cancel-button:hover {
  background-color: #e0e0e0;
}

.confirm-button {
  background-color: #2196f3;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 8px 16px;
  font-size: 14px;
  cursor: pointer;
  transition: background-color 0.2s;
}

.confirm-button:hover {
  background-color: #1976d2;
}
</style>
