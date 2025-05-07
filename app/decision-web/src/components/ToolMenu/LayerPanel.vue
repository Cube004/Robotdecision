<template>
  <div class="layer-panel" v-if="visible">
    <div class="panel-header">
      <h3>图层管理</h3>
      <button class="close-button" @click="$emit('close')" v-html="iconSvg.close">
      </button>
    </div>

    <div class="panel-content">
      <div class="layer-item" v-for="(layer, index) in layers" :key="index">
        <div class="layer-info">
          <div class="layer-icon" :style="{ backgroundColor: layer.color }">
            <div v-html="getLayerIcon(layer.type)"></div>
          </div>
          <span class="layer-name">{{ layer.name }}</span>
        </div>

        <div class="layer-controls">
          <button class="visibility-toggle" :class="{ 'visible': layer.visible }" @click="toggleLayerVisibility(index)" v-html="layer.visible ? iconSvg.visible : iconSvg.hidden">
          </button>
          <button class="lock-toggle" :class="{ 'locked': layer.locked }" @click="toggleLayerLock(index)" v-if="false" v-html="layer.locked ? iconSvg.locked : iconSvg.unlocked">
          </button>
        </div>
      </div>
    </div>

    <div class="panel-footer" v-if="false">
      <button class="add-layer-button">
        <div v-html="iconSvg.add"></div>
        <span>添加图层</span>
      </button>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref } from 'vue';
import { layers } from '@/types/Layers';
// 接收传入的props
defineProps({
  visible: {
    type: Boolean,
    required: true
  }
});

// 所有SVG图标
const iconSvg = ref({
  // 关闭按钮
  close: `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M18 6L6 18" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    <path d="M6 6L18 18" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
  </svg>`,

  // 图层类型图标
  node: `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <rect x="3" y="4" width="18" height="14" rx="3" stroke="white" stroke-width="2"/>
  </svg>`,

  edge: `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M3 20L21 4" stroke="white" stroke-width="2" stroke-linecap="round"/>
  </svg>`,

  group: `<svg t="1746630585972" width="14" height="14"  viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="4305">
      <path fill="white" d="M875.76 751.32H771.98c-18.75 0-33.96-15.2-33.96-33.96s15.2-33.96 33.96-33.96h103.78c8.3 0 15.31-8.21 15.31-17.93V190.71c0-9.72-7.01-17.93-15.31-17.93H301.29c-8.3 0-15.31 8.21-15.31 17.93v115.93c0 18.75-15.2 33.96-33.96 33.96s-33.96-15.2-33.96-33.96V190.71c0-47.34 37.34-85.85 83.23-85.85h574.47c45.89 0 83.23 38.51 83.23 85.85v474.76c0 47.34-37.34 85.85-83.23 85.85z"  p-id="4306"></path>
      <path fill="white" d="M722.71 919.14H148.24c-45.89 0-83.23-38.51-83.23-85.85V358.53c0-47.34 37.34-85.85 83.23-85.85h574.47c45.89 0 83.23 38.51 83.23 85.85v474.76c0 47.34-37.34 85.85-83.23 85.85zM148.24 340.6c-8.3 0-15.31 8.21-15.31 17.93v474.76c0 9.72 7.01 17.93 15.31 17.93h574.47c8.3 0 15.31-8.21 15.31-17.93V358.53c0-9.72-7.01-17.93-15.31-17.93H148.24z"  p-id="4307">
    </path>
    </svg>`,

  // 默认图标
  other: `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <circle cx="12" cy="12" r="8" stroke="white" stroke-width="2"/>
  </svg>`,

  background: `<svg t="1746632388097" class="icon" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="9341" width="14" height="14">
  <path d="M950.857 160.914a87.771 87.771 0 0 0-87.771-87.771H160.914a87.771 87.771 0 0 0-87.771 87.771v702.172a87.771 87.771 0 0 0 87.771 87.771h702.172a87.771 87.771 0 0 0 87.771-87.771V160.914z m58.514 0v702.172a146.286 146.286 0 0 1-146.285 146.285H160.914A146.286 146.286 0 0 1 14.629 863.086V160.914A146.286 146.286 0 0 1 160.914 14.63h702.172a146.286 146.286 0 0 1 146.284 146.284z" fill="white" p-id="9342"></path>
  <path d="M842.533 610.45a29.257 29.257 0 1 1 41.106 41.619L649.582 883.93a29.257 29.257 0 1 1-41.107-41.691L842.533 610.45zM374.418 140.142a29.257 29.257 0 1 1 41.107 41.545L181.467 413.55a29.257 29.257 0 0 1-41.106-41.619L374.418 140.07zM842.46 377.856a29.257 29.257 0 1 1 41.253 41.545L415.599 883.858a29.257 29.257 0 1 1-41.253-41.545L842.46 377.856zM608.402 140.142a29.257 29.257 0 1 1 41.253 41.545L181.54 646.144a29.257 29.257 0 1 1-41.253-41.545l468.115-464.457z m204.727 29.33a29.257 29.257 0 1 1 41.399 41.399L210.871 854.528a29.257 29.257 0 0 1-41.399-41.399l643.657-643.657z" fill="white" p-id="9343"></path>
  </svg>`,


  area: `<svg t="1746547363258" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg" p-id="18517" width="14" height="14">
      <path fill="white" d="M333.226667 197.290667l16.725333-83.669334 376.533333 75.178667-16.682666 83.669333zM790.698667 351.872l84.736-10.154667 40.533333 338.901334-84.736 10.154666zM227.285333 831.530667l559.189334-66.816 10.154666 84.736-559.232 66.816zM111.530667 782.08L196.224 225.365333l84.352 12.8-84.693333 556.8z" p-id="18518"></path>
      <path fill="white" d="M247.466667 277.333333C170.666667 277.333333 106.666667 213.333333 106.666667 136.533333S170.666667 0 247.466667 0 384 64 384 136.533333 324.266667 277.333333 247.466667 277.333333z m0-192c-29.866667 0-55.466667 25.6-55.466667 51.2s25.6 51.2 51.2 51.2S298.666667 166.4 298.666667 136.533333 277.333333 85.333333 247.466667 85.333333zM136.533333 1024C64 1024 0 960 0 887.466667s64-136.533333 136.533333-136.533334c76.8 0 136.533333 64 136.533334 136.533334S213.333333 1024 136.533333 1024z m0-192c-29.866667 0-51.2 25.6-51.2 51.2S110.933333 938.666667 136.533333 938.666667s51.2-25.6 51.2-51.2-21.333333-55.466667-51.2-55.466667zM810.666667 392.533333c-76.8 0-136.533333-64-136.533334-136.533333s64-136.533333 136.533334-136.533333 136.533333 64 136.533333 136.533333-59.733333 136.533333-136.533333 136.533333z m0-192c-29.866667 0-51.2 25.6-51.2 51.2s25.6 51.2 51.2 51.2 51.2-25.6 51.2-51.2-21.333333-51.2-51.2-51.2zM887.466667 917.333333c-76.8 0-136.533333-64-136.533334-136.533333S810.666667 640 887.466667 640s136.533333 64 136.533333 136.533333-64 140.8-136.533333 140.8z m0-192c-29.866667 0-51.2 25.6-51.2 51.2s25.6 51.2 51.2 51.2S938.666667 810.666667 938.666667 780.8s-25.6-55.466667-51.2-55.466667z" p-id="18519"></path>
    </svg>`,
  // 可见性图标
  visible: `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M1 12C1 12 5 4 12 4C19 4 23 12 23 12C23 12 19 20 12 20C5 20 1 12 1 12Z" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    <circle cx="12" cy="12" r="3" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
  </svg>`,

  hidden: `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M2 2L22 22" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    <path d="M6.71277 6.7226C3.66479 8.79527 2 12 2 12C2 12 5.63636 19 12 19C14.0503 19 15.8174 18.2734 17.2711 17.2884M11 5.05822C11.3254 5.02013 11.6588 5 12 5C18.3636 5 22 12 22 12C22 12 21.3082 13.3317 20 14.8335" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    <path d="M14 14.2362C13.4692 14.7112 12.7684 15.0001 12 15.0001C10.3431 15.0001 9 13.657 9 12.0001C9 11.1764 9.33193 10.4303 9.86932 9.88818" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
  </svg>`,

  // 锁定图标
  locked: `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <rect x="5" y="11" width="14" height="10" rx="2" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    <path d="M7 11V7C7 4.23858 9.23858 2 12 2C14.7614 2 17 4.23858 17 7V11" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
  </svg>`,

  unlocked: `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <rect x="5" y="11" width="14" height="10" rx="2" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    <path d="M17 11V7C17 4.23858 14.7614 2 12 2C10.8002 2 9.68758 2.40856 8.80373 3.09814" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    <path d="M7 11V7C7 6.27143 7.18 5.58943 7.5 5" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
  </svg>`,

  // 添加图层按钮
  add: `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 5V19" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
    <path d="M5 12H19" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
  </svg>`
});

// 模拟图层数据


// 根据图层类型获取对应图标
const getLayerIcon = (type: string) => {
  return iconSvg.value[type as keyof typeof iconSvg.value] || iconSvg.value.other;
};

// 切换图层可见性
const toggleLayerVisibility = (index: number) => {
  layers.value[index].visible = !layers.value[index].visible;
};

// 切换图层锁定状态
const toggleLayerLock = (index: number) => {
  layers.value[index].locked = !layers.value[index].locked;
};
</script>

<style scoped>
.layer-panel {
  position: fixed;
  right: 24px;
  top: 80px;
  width: 260px;
  background-color: white;
  border-radius: 8px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  z-index: 1000;
  display: flex;
  flex-direction: column;
  overflow: hidden;
  transition: all 0.3s ease;
}

.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  border-bottom: 1px solid #E2E8F0;
}

.panel-header h3 {
  margin: 0;
  font-size: 14px;
  font-weight: 600;
  color: #1E293B;
}

.close-button {
  background: transparent;
  border: none;
  cursor: pointer;
  padding: 4px;
  border-radius: 4px;
  color: #64748B;
  display: flex;
  align-items: center;
  justify-content: center;
}

.close-button:hover {
  background-color: #F1F5F9;
  color: #1E293B;
}

.panel-content {
  padding: 8px 0;
  max-height: 400px;
  overflow-y: auto;
}

.layer-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 16px;
  transition: background-color 0.2s ease;
}

.layer-item:hover {
  background-color: #F8FAFC;
}

.layer-info {
  display: flex;
  align-items: center;
  gap: 8px;
}

.layer-icon {
  width: 24px;
  height: 24px;
  border-radius: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.layer-name {
  font-size: 13px;
  color: #334155;
}

.layer-controls {
  display: flex;
  align-items: center;
  gap: 8px;
}

.visibility-toggle, .lock-toggle {
  padding: 4px;
  border: none;
  background: transparent;
  color: #94A3B8;
  cursor: pointer;
  border-radius: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s ease;
}

.visibility-toggle:hover, .lock-toggle:hover {
  background-color: #F1F5F9;
  color: #475569;
}

.visibility-toggle.visible {
  color: #3B82F6;
}

.lock-toggle.locked {
  color: #F59E0B;
}

.panel-footer {
  padding: 12px 16px;
  border-top: 1px solid #E2E8F0;
}

.add-layer-button {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 12px;
  background-color: #F1F5F9;
  border: none;
  border-radius: 6px;
  color: #475569;
  font-size: 13px;
  cursor: pointer;
  transition: all 0.2s ease;
  width: 100%;
  justify-content: center;
}

.add-layer-button:hover {
  background-color: #E2E8F0;
  color: #1E293B;
}
</style>
