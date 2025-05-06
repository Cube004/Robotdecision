<template>
  <div class="add-edge-container" v-if="visible">
    <!-- 顶部提示栏 -->
    <div class="tip-bar" :class="{ 'confirmation-active': showConfirmation }">
      <div v-if="!showConfirmation" class="tip-content">
        <div class="tip-icon">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M18.7491 7.72104L11.525 7.72104L11.525 9.62563L18.7491 9.62563L15.6132 12.7615C15.5222 12.8494 15.4497 12.9545 15.3998 13.0706C15.3499 13.1868 15.3236 13.3118 15.3225 13.4382C15.3214 13.5647 15.3455 13.6901 15.3934 13.8071C15.4413 13.9241 15.512 14.0305 15.6014 14.1199C15.6908 14.2093 15.7971 14.28 15.9141 14.3279C16.0312 14.3758 16.1566 14.3999 16.283 14.3988C16.4095 14.3977 16.5344 14.3714 16.6506 14.3215C16.7668 14.2716 16.8719 14.199 16.9597 14.1081L21.7212 9.3466C21.8997 9.16802 22 8.92585 22 8.67333C22 8.42082 21.8997 8.17864 21.7212 8.00006L16.9597 3.2386C16.8719 3.14765 16.7668 3.0751 16.6506 3.02519C16.5344 2.97528 16.4095 2.94901 16.283 2.94791C16.1566 2.94681 16.0312 2.97091 15.9141 3.01879C15.7971 3.06667 15.6908 3.13738 15.6014 3.2268C15.512 3.31621 15.4413 3.42254 15.3934 3.53957C15.3455 3.6566 15.3214 3.782 15.3225 3.90844C15.3236 4.03489 15.3499 4.15985 15.3998 4.27603C15.4497 4.39221 15.5222 4.4973 15.6132 4.58514L18.7491 7.72104Z" fill="currentColor"></path>
            <path d="M11.5229 9.62454C10.997 9.62454 10.5706 10.0509 10.5706 10.5768V18.1952C10.5706 19.773 9.29156 21.052 7.71375 21.052H2.95229C2.42636 21.052 2 20.6257 2 20.0998C2 19.5738 2.42636 19.1475 2.95229 19.1475H7.71375C8.23969 19.1475 8.66605 18.7211 8.66605 18.1952V10.5768C8.66605 8.99902 9.94511 7.71996 11.5229 7.71996V9.62454Z" fill="currentColor"></path>
          </svg>
        </div>
        <div class="tip-text">
          {{ step === 1 ? '请选择起点节点' : '请选择终点节点' }}
        </div>
      </div>

      <!-- 确认添加提示 -->
      <div v-if="showConfirmation" class="confirmation-prompt">
        <span>确认添加连接线？</span>
        <div class="confirmation-actions">
          <button class="confirm-btn" @click="confirmAddEdge">确认</button>
          <button class="cancel-btn" @click="cancelAddEdge">取消</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, defineProps } from 'vue';
import {
  handleConfirmAddEdge, handleCancelAddEdge,
  selectedStartNodeId, selectedEndNodeId
} from '@/types/extensions/ToolMenu/AddEdge';

defineProps<{
  visible: boolean;
}>();

const step = ref(1); // 1: 选择起点, 2: 选择终点
const showConfirmation = ref(false);

// 监听起点和终点的变化
watch(() => selectedStartNodeId.value, (newVal) => {
  if (newVal !== null && newVal !== undefined) {
    step.value = 2; // 已选择起点，下一步选择终点
  }
});

watch(() => selectedEndNodeId.value, (newVal) => {
  if (newVal !== null && newVal !== undefined) {
    showConfirmation.value = true; // 显示确认提示
  }
});

// 确认添加连接线
const confirmAddEdge = () => {
  handleConfirmAddEdge({
    startNodeId: selectedStartNodeId.value as number,
    endNodeId: selectedEndNodeId.value as number
  });
  resetState();
};

// 取消添加连接线
const cancelAddEdge = () => {
  handleCancelAddEdge();
  resetState();
};

// 重置状态
const resetState = () => {
  step.value = 1;
  showConfirmation.value = false;
};
</script>

<style scoped>
.add-edge-container {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
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
</style>
