<template>
  <div class="map-overlay" v-if="visible" :class="{ visible: showMap }">
    <div class="map-container">
      <div class="map-image-div" ref="mapContainer">
        <div class="map-content-wrapper">
          <img class="map-image" ref="mapImage" src="/map/RUMC.png" alt="航点地图" @click="handleMapClick">
          <!-- 渲染所有航点 -->
          <div
            v-for="point in points"
            :key="point.id.value"
            class="map-point"
            :style="{
              left: `${point.position.x}px`,
              top: `${point.position.y}px`,
              backgroundColor: point.color.value,
              position: 'absolute'
            }"
            :class="{ active: selectedPoint && selectedPoint.id.value === point.id.value }"
            @click.stop="selectPointById(point.id.value)"
          >
            <div class="point-text" v-if="point.text.value" :style="{ fontSize: point.fontSize.value + 'px', color: point.textColor.value }">
              {{ point.text.value }}
            </div>

            <div class="point-waypoint-text" v-if="point.waypoint" :style="{ fontSize: point.fontSize.value + 'px', color: point.textColor.value }">
              {{ point.waypoint.x.toFixed(1) }}, {{ point.waypoint.y.toFixed(1) }}
            </div>
          </div>
          <!-- 渲染地图设置点 -->
          <div
            v-for="point in MapSettingsPoints"
            :key="point.id.value"
            class="map-point"
            :style="{
              left: `${point.position.x}px`,
              top: `${point.position.y}px`,
              backgroundColor: point.color.value,
              position: 'absolute'
            }"
          >
            <div class="point-text" v-if="point.text.value" :style="{ fontSize: point.fontSize.value + 'px', color: point.textColor.value }">
              {{ point.text.value }}
            </div>
          </div>
          <!-- 渲染预览点 -->
          <div
            v-if="previewPoints !== null"
            :key="previewPoints.id.value"
            class="map-point"
            :style="{ // 使用鼠标位置
              left: `${mousePosition.x}px`,
              top: `${mousePosition.y}px`,
              backgroundColor: previewPoints.color.value,
              position: 'absolute'
            }"
            :class="{ active: selectedPoint && selectedPoint.id.value === previewPoints.id.value }"
            @click.stop="selectPointById(previewPoints.id.value)"
          >
            <div class="point-text" v-if="previewPoints.text.value" :style="{ fontSize: previewPoints.fontSize.value + 'px', color: previewPoints.textColor.value }">
              {{ previewPoints.text.value }}
            </div>
          </div>
        </div>

        <!-- 航点菜单 -->
        <div class="menu point-menu" id="pointMenu" v-show="showPointMenu" :style="pointMenuStyle">
          <div class="menu-item" id="pointColor">
            <button id="point-color-button" @click="toggleColorPicker">
              <div class="icon">
                <svg style="fill: #666" width="18" height="18" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg">
                  <path d="M512 512m-512 0a512 512 0 1 0 1024 0 512 512 0 1 0-1024 0Z"></path>
                </svg>
              </div>
              <div class="menu-text">颜色</div>
              <div class="arrow">
                <svg width="8" height="8" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="m3.414 7.086-.707.707a1 1 0 0 0 0 1.414l7.778 7.778a2 2 0 0 0 2.829 0l7.778-7.778a1 1 0 0 0 0-1.414l-.707-.707a1 1 0 0 0-1.415 0l-7.07 7.07-7.072-7.07a1 1 0 0 0-1.414 0Z" fill="currentColor"></path>
                </svg>
              </div>
            </button>

            <div class="color-picker-content" v-show="showColorPicker">
              <div class="color-grid">
                <div
                  v-for="color in colorOptions"
                  :key="color"
                  class="color-swatch"
                  :class="{ selected: selectedColor === color }"
                  :style="{ backgroundColor: color }"
                  :data-color="color"
                  @click="selectColor(color)"
                ></div>
                <div class="color-option add" @click="toggleCustomColor">+</div>
              </div>

              <div class="custom-color-section" v-show="showCustomColor">
                <div class="custom-color-header">自定义颜色</div>
                <div class="custom-color-input">
                  <input type="color" v-model="customColor">
                  <input type="text" v-model="customColor">
                </div>
                <button id="addCustomColorButton" @click="addCustomColor">添加颜色</button>
              </div>
            </div>
          </div>

          <div class="menu-item" id="textProperties">
            <button id="textProperties-button" @click="toggleTextProperties">
              <div class="icon">
                <svg style="fill: #666" width="20" height="20" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg">
                  <path d="M747.24 345.95c-11.47 0-20.76-9.3-20.76-20.76v-20.76H297.51v20.76c0 11.46-9.29 20.76-20.76 20.76S256 336.65 256 325.19v-41.51c0-11.46 9.29-20.76 20.76-20.76h470.49c11.47 0 20.76 9.3 20.76 20.76v41.51c-0.01 11.46-9.3 20.76-20.77 20.76z"></path>
                  <path d="M512 761.08c-11.47 0-20.76-9.3-20.76-20.76V283.68c0-11.46 9.29-20.76 20.76-20.76 11.47 0 20.76 9.3 20.76 20.76v456.65c0 11.45-9.29 20.75-20.76 20.75z"></path>
                  <path d="M581.19 761.08H442.81c-11.47 0-20.76-9.3-20.76-20.76s9.29-20.76 20.76-20.76h138.38c11.47 0 20.76 9.3 20.76 20.76s-9.29 20.76-20.76 20.76z"></path>
                </svg>
              </div>
              <div class="menu-text">文本</div>
              <div class="arrow">
                <svg width="8" height="8" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="m3.414 7.086-.707.707a1 1 0 0 0 0 1.414l7.778 7.778a2 2 0 0 0 2.829 0l7.778-7.778a1 1 0 0 0 0-1.414l-.707-.707a1 1 0 0 0-1.415 0l-7.07 7.07-7.072-7.07a1 1 0 0 0-1.414 0Z" fill="currentColor"></path>
                </svg>
              </div>
            </button>

            <div class="text-properties-content" v-show="showTextProperties">
              <div class="text-input-section">
                <div class="text-input-label">文本内容</div>
                <textarea class="text-input-area" v-model="pointText" rows="3" placeholder="请输入文本"></textarea>
              </div>

              <div class="text-style-section">
                <div class="text-size-control">
                  <div class="control-label">字体大小</div>
                  <div class="size-input-group">
                    <input type="number" class="font-size" v-model="fontSize" min="8" max="72">
                    <span class="unit">px</span>
                  </div>
                </div>

                <div class="text-color-control">
                  <div class="control-label">文本颜色</div>
                  <div class="color-grid text-color-grid">
                    <div
                      v-for="color in textColorOptions"
                      :key="color"
                      class="color-option"
                      :class="{ selected: selectedTextColor === color }"
                      :style="{ backgroundColor: color }"
                      :data-color="color"
                      @click="selectTextColor(color)"
                    ></div>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div class="menu-item" id="pointDelete">
            <button id="point-delete-button" @click="toggleDeleteConfirm">
              <div class="icon">
                <svg style="fill: #666" width="18" height="18" viewBox="0 0 1024 1024" version="1.1" xmlns="http://www.w3.org/2000/svg">
                  <path d="M834.24768 256c-14.15168 0-25.6 11.08992-25.6 24.79616v668.80512c0 13.66016-11.47392 24.80128-25.6 24.80128H226.74944c-14.12608 0-25.6-11.14112-25.6-24.80128V280.79616c0-13.70624-11.44832-24.79616-25.6-24.79616s-25.6 11.08992-25.6 24.79616v668.80512c0 41.02656 34.45248 74.39872 76.8 74.39872h556.29824c42.34752 0 76.8-33.37216 76.8-74.39872V280.79616c0-13.70624-11.44832-24.79616-25.6-24.79616zM336.21504 51.2h353.28a25.6 25.6 0 0 0 0-51.2h-353.28a25.6 25.6 0 0 0 0 51.2z"></path>
                  <path d="M433.06496 846.99136v-558.08a25.6 25.6 0 0 0-51.2 0v558.08a25.6 25.6 0 0 0 51.2 0zM636.3392 846.99136v-558.08a25.6 25.6 0 0 0-51.2 0v558.08a25.6 25.6 0 0 0 51.2 0zM970.24 124.58496h-916.48a25.6 25.6 0 0 0 0 51.2h916.48a25.6 25.6 0 0 0 0-51.2z"></path>
                </svg>
              </div>
              <div class="menu-text">删除</div>
              <div class="arrow">
                <svg width="8" height="8" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="m3.414 7.086-.707.707a1 1 0 0 0 0 1.414l7.778 7.778a2 2 0 0 0 2.829 0l7.778-7.778a1 1 0 0 0 0-1.414l-.707-.707a1 1 0 0 0-1.415 0l-7.07 7.07-7.072-7.07a1 1 0 0 0-1.414 0Z" fill="currentColor"></path>
                </svg>
              </div>
            </button>

            <div class="delete-confirm-content" v-show="showDeleteConfirm">
              <div class="delete-message">
                <div class="warning-icon">
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M12 22c5.523 0 10-4.477 10-10S17.523 2 12 2 2 6.477 2 12s4.477 10 10 10zm0-7a1 1 0 1 0 0 2 1 1 0 0 0 0-2zm0-8a1 1 0 0 0-1 1v5a1 1 0 1 0 2 0V8a1 1 0 0 0-1-1z" fill="#f44336"></path>
                  </svg>
                </div>
                <div class="message-text">
                  <div class="message-title">删除航点</div>
                  <div class="message-desc">此操作无法撤销</div>
                </div>
              </div>
              <button class="confirm-button" id="deletePointButton" @click="deletePoint">删除</button>
            </div>
          </div>
        </div>

        <!-- 悬浮功能菜单 -->
        <div
          class="floating-menu"
          :class="{ dragging: isDragging }"
          :style="{ top: menuPosition.top + 'px', left: menuPosition.left + 'px' }"
          v-show="showMapTools"
        >
          <div
            class="floating-menu-header"
          >
            <div class="header-title">
              <!-- <svg viewBox="0 0 24 24" width="16" height="16" class="header-icon">
                <path d="M12 3L1 9l11 6 9-4.91V17h2V9L12 3M5 14.99v-2.23l6 3.24 6-3.24V15l-6 3.24-6-3.24z" fill="currentColor"/>
              </svg> -->
              <span>地图工具</span>
            </div>
            <button class="floating-menu-toggle" :class="{ collapsed: !showFloatingMenu }" @click="()=>{showFloatingMenu = !showFloatingMenu}" title="折叠/展开菜单">
              <svg viewBox="0 0 24 24" width="14" height="14">
                <path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z"></path>
              </svg>
            </button>
          </div>

          <div class="floating-menu-content" v-show="showFloatingMenu">
            <div class="menu-section">
              <div class="section-title">初始化启动区</div>
              <button class="tool-button" @click="setStartingArea" data-tooltip="设置机器人启动区域">
                <div class="button-icon">
                  <svg viewBox="0 0 24 24" width="16" height="16">
                    <path d="M12 2L4.5 20.29l.71.71L12 18l6.79 3 .71-.71z" fill="currentColor"></path>
                  </svg>
                </div>
                <span>放置启动区</span>
              </button>
            </div>

            <div class="menu-section">
              <div class="section-title">边界点</div>
              <div class="buttons-row">
                <button class="tool-button boundary-button" @click="setTopLeftCorner" data-tooltip="放置地图左上角边界点">
                  <div class="button-icon">
                    <svg viewBox="0 0 24 24" width="16" height="16">
                      <path d="M4 4h6v6H4V4z M4 4L10 4 10 10 4 10z" fill="currentColor"></path>
                    </svg>
                  </div>
                  <span>左上角</span>
                </button>
                <button class="tool-button boundary-button" @click="setBottomRightCorner" data-tooltip="放置地图右下角边界点">
                  <div class="button-icon">
                    <svg viewBox="0 0 24 24" width="16" height="16">
                      <path d="M14 14h6v6h-6v-6z M14 14L20 14 20 20 14 20z" fill="currentColor"></path>
                    </svg>
                  </div>
                  <span>右下角</span>
                </button>
              </div>
            </div>

            <div class="menu-section">
              <div class="section-title">地图尺寸</div>
              <div class="input-row">
                <div class="input-group">
                  <label for="map-width">长度 (m)</label>
                  <input type="number" id="map-width" class="size-input" v-model="mapWidth" min="1" step="1">
                </div>
                <div class="input-group">
                  <label for="map-height">宽度 (m)</label>
                  <input type="number" id="map-height" class="size-input" v-model="mapHeight" min="1" step="1">
                </div>
              </div>
              <button class="tool-button apply-button" @click="applyMapSize" data-tooltip="应用尺寸设置到地图">
                <div class="button-icon">
                  <svg viewBox="0 0 24 24" width="16" height="16">
                    <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z" fill="currentColor"></path>
                  </svg>
                </div>
                <span>应用设置</span>
              </button>
            </div>
          </div>
        </div>
      </div>

      <!-- 关闭按钮 -->
      <button class="close-button" @click="closeMap">
        <svg viewBox="0 0 24 24" width="14" height="14">
          <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"></path>
        </svg>
      </button>

      <!-- 状态栏 -->
      <div class="map-status-bar">
        <div class="status-section">
          <div class="status-icon">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M12 22C17.5228 22 22 17.5228 22 12C22 6.47715 17.5228 2 12 2C6.47715 2 2 6.47715 2 12C2 17.5228 6.47715 22 12 22Z" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
              <path d="M12 16V12" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
              <path d="M12 8H12.01" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
            </svg>
          </div>
          <div class="status-text">左键选择航点 | Ctrl+左键创建航点</div>
        </div>

        <div class="status-section" v-if="selectedPoint">
          <div class="status-text">当前选中航点 ID: {{ selectedPoint.id.value }}</div>
        </div>

        <div class="status-section map-coordinates" v-if="mousePosition.x && mousePosition.y">
          <div class="status-text">坐标: {{ mousePosition.x }}, {{ mousePosition.y }}</div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts" name="MapPage">
import { onMounted, defineProps, watch, onUnmounted, watchEffect } from 'vue';
import { selectedPoint, pointText, fontSize, selectedTextColor } from '@/types/extensions/ToolMenu/PointMenu';
import {
  closeMap,
  showMap,
  handleMapClick,
  pointMenuStyle,
  showPointMenu,
  mapContainer
} from '@/types/extensions/ToolMenu/MapPage';
import {
  toggleColorPicker,
  toggleCustomColor,
  selectColor,
  addCustomColor,
  toggleTextProperties,
  selectTextColor,
  toggleDeleteConfirm,
  deletePoint,
  selectPointById,
  colorOptions,
  showColorPicker,
  showCustomColor,
  showTextProperties,
  showDeleteConfirm,
  textColorOptions,
  selectedColor,
  customColor,
  mapImage
} from '@/types/extensions/ToolMenu/PointMenu';
import {
  showFloatingMenu,
  showMapTools,
  isDragging,
  menuPosition,
  setStartingArea,
  setTopLeftCorner,
  setBottomRightCorner,
  applyMapSize,
  previewPoints,
  mousePosition
} from '@/types/extensions/ToolMenu/MapPage';
import { points } from '@/types/Manger';
import { mapWidth, mapHeight, MapSettingsPoints } from '@/types/Manger';
// 定义 props
const props = defineProps<{
  visible: boolean;
}>();


// 监听 visible 属性变化
watch(() => props.visible, (newVal) => {
  if (newVal) {
    setTimeout(() => {
      showMap.value = true;
    }, 50);
  } else {
    showMap.value = false;
  }
});

// 跟踪鼠标位置
const trackMousePosition = (event: MouseEvent) => {
  if (!mapContainer.value) return;

  const rect = mapContainer.value.getBoundingClientRect();
  mousePosition.value = {
    x: Math.round(event.clientX - rect.left),
    y: Math.round(event.clientY - rect.top)
  };
};

// 处理右键菜单和鼠标位置跟踪
onMounted(() => {
  const stop = watchEffect(() => {
    const container = mapContainer.value;
    if (container) {
      container.addEventListener('contextmenu', (e) => {
        e.preventDefault();
      });
      container.addEventListener('mousemove', trackMousePosition);
      stop();
    }
  });
});

// 移除事件监听器
onUnmounted(() => {
  if (mapContainer.value) {
    mapContainer.value.removeEventListener('mousemove', trackMousePosition);
  }
});
</script>

<style scoped>
.map-overlay {
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
  opacity: 0;
  transition: opacity 0.3s ease;
}

.map-overlay.visible {
  opacity: 1;
}

.map-container {
  position: relative;
  width: 85%;
  max-width: 1400px;
  height: 85%;
  background-color: #f8fafc;
  border-radius: 12px;
  overflow: visible; /* 确保弹出框不被裁剪 */
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.08);
  display: flex;
  flex-direction: column;
}

.map-image-div {
  position: relative;
  flex: 1;
  background-color: #fff;
  overflow: hidden; /* 仅图像区域的滚动条隐藏 */
  border-radius: 12px 12px 0 0; /* 保持顶部圆角 */
  box-shadow: inset 0 0 10px rgba(0, 0, 0, 0.05); /* 内阴影效果 */
  display: flex;
  justify-content: center;
  align-items: center;
}

.map-content-wrapper {
  position: relative;
  display: inline-block; /* 关键：使容器尺寸依据图片内容 */
  max-width: 100%;
  max-height: 100%;
  transform-origin: center; /* 确保从中心点缩放 */
}

.map-image {
  display: block;
  max-width: 100%;
  max-height: 100%;
  width: auto;
  height: auto;
  object-fit: contain;
  cursor: crosshair;
}

/* 航点样式 */
.map-point {
  position: absolute;
  width: 12px;
  height: 12px;
  border-radius: 50%;
  transform: translate(-50%, -50%);
  cursor: pointer;
  z-index: 10;
  box-shadow: 0 0 0 2px white;
  transition: transform 0.2s ease;
}

/* 增加航点点击区域，提升用户体验 */
.map-point::before {
  content: '';
  position: absolute;
  width: 24px;
  height: 24px;
  border-radius: 50%;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  z-index: -1;
}

.map-point.active {
  transform: translate(-50%, -50%) scale(1.5);
  z-index: 11;
}

.point-text {
  position: absolute;
  top: -20px;
  left: 50%;
  transform: translateX(-50%);
  white-space: nowrap;
  background-color: rgba(255, 255, 255, 0.8);
  padding: 2px 6px;
  border-radius: 4px;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  pointer-events: none;
  z-index: 12;
}

.point-waypoint-text {
  position: absolute;
  top: 10px;
  left: 50%;
  transform: translateX(-50%);
  white-space: nowrap;
  background-color: rgba(255, 255, 255, 0.4);
  padding: 2px 6px;
  border-radius: 4px;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
  pointer-events: none;
  z-index: 12;
}

/* 关闭按钮样式 */
.close-button {
  position: absolute;
  top: -18px;
  right: -18px;
  background-color: #f43f5e; /* 使用醒目的红色 */
  color: white;
  border: 2px solid white; /* 白色边框增加凸显效果 */
  border-radius: 50%;
  width: 36px;
  height: 36px;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  box-shadow: 0 3px 12px rgba(0, 0, 0, 0.2); /* 增强阴影 */
  transition: all 0.2s ease;
  z-index: 2001;
  padding: 0;
  transform: translate(0, 0);
}

.close-button:hover {
  background-color: #e11d48; /* 深红色悬停效果 */
  transform: scale(1.05);
}

.close-button svg {
  width: 18px;
  height: 18px;
  fill: white; /* 白色图标 */
}

/* 状态栏样式 */
.map-status-bar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
  height: 38px;
  background-color: #f8fafc;
  border-top: 1px solid #e2e8f0;
  border-radius: 0 0 12px 12px;
  font-size: 13px;
  color: #64748b;
  flex-wrap: wrap;
  box-shadow: 0 -1px 2px rgba(0, 0, 0, 0.03);
}

.status-section {
  display: flex;
  align-items: center;
  gap: 4px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  max-width: 50%;
  font-weight: 500;
}

.map-coordinates {
  max-width: 150px;
  overflow: hidden;
  text-overflow: ellipsis;
}

.map-status-icon {
  color: #3b82f6;
  font-size: 16px;
  margin-right: 4px;
}

/* 点菜单样式 */
.menu {
  position: absolute;
  background-color: #ffffff;
  border-radius: 8px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  padding: 10px;
  min-width: 250px;
  max-width: 280px;
  z-index: 1000;
  overflow: visible;
  border: 1px solid #e2e8f0;
}

.menu.point-menu {
  width: 280px; /* 设置固定宽度与子菜单一致 */
  max-width: 280px;
  min-width: 280px; /* 确保最小宽度也一致 */
  box-sizing: border-box; /* 确保padding和border计入宽度 */
}

.menu-item {
  position: relative;
  margin-bottom: 6px;
  overflow: visible; /* 确保子菜单可见 */
}

.menu-item:last-child {
  margin-bottom: 0;
}

.menu-item button {
  display: flex;
  align-items: center;
  width: 100%;
  padding: 10px 12px;
  background-color: transparent;
  border: none;
  border-radius: 6px;
  cursor: pointer;
  transition: background-color 0.2s;
  font-weight: 500;
}

.menu-item button:hover {
  background-color: #f1f5f9;
}

.icon {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 24px;
  height: 24px;
  margin-right: 12px;
}

.menu-text {
  flex: 1;
  text-align: left;
  font-size: 14px;
  color: #334155;
}

.arrow {
  display: flex;
  align-items: center;
}

.color-picker-content,
.text-properties-content,
.delete-confirm-content {
  background-color: #ffffff;
  border-radius: 8px;
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.12);
  padding: 16px;
  margin-top: 8px;
  width: 100%;
  max-width: 100%;
  box-sizing: border-box;
  z-index: 1010;
  transform-origin: top center;
  transition: all 0.2s ease-out;
}

.color-picker-content {
  border: 1px solid #e2e8f0;
  padding: 12px;
}

.color-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(28px, 1fr));
  gap: 10px;
  margin-bottom: 12px;
  width: 100%;
  max-width: 100%;
}

.color-swatch {
  width: 28px;
  height: 28px;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s ease;
  border: 2px solid transparent;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
}

.color-swatch:hover {
  transform: scale(1.1);
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.15);
}

.color-swatch.selected {
  border: 2px solid #fff;
  box-shadow: 0 0 0 2px #3b82f6, 0 2px 5px rgba(0, 0, 0, 0.1);
}

.text-color-grid {
  grid-template-columns: repeat(5, 1fr);
}

.color-option {
  width: 24px;
  height: 24px;
  border-radius: 50%;
  cursor: pointer;
  border: 2px solid transparent;
  transition: all 0.2s;
}

.color-option:hover {
  transform: scale(1.1);
}

.color-option.selected {
  border-color: #3b82f6;
  transform: scale(1.1);
}

.color-option.add {
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #f8f9fa;
  color: #5f6368;
  font-weight: bold;
  font-size: 16px;
}

.custom-color-section {
  border-top: 1px solid #e2e8f0;
  padding-top: 12px;
  margin-top: 4px;
  width: 100%;
}

.custom-color-header {
  font-size: 14px;
  color: #64748b;
  margin-bottom: 10px;
  font-weight: 500; /* 稍微加粗标题 */
}

.custom-color-input {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 10px;
  width: 100%;
}

.custom-color-input input {
  flex-grow: 1;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  padding: 8px 10px;
  font-size: 14px;
  max-width: calc(100% - 40px);
}

.color-preview {
  width: 30px;
  height: 30px;
  border-radius: 6px;
  border: 1px solid #e2e8f0;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
}

.button {
  background-color: #3b82f6;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 10px 12px; /* 增加按钮高度 */
  font-size: 14px;
  font-weight: 600; /* 加粗文字 */
  cursor: pointer;
  width: 100%;
  transition: all 0.2s ease;
  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.2); /* 添加文字阴影增加可读性 */
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1); /* 添加轻微阴影 */
  letter-spacing: 0.5px; /* 增加字间距 */
}

.button:hover {
  background-color: #2563eb;
  transform: translateY(-1px); /* 悬停时轻微上移 */
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.15); /* 增强悬停时的阴影 */
}

.text-input-section {
  margin-bottom: 12px;
  width: 100%;
}

.text-input-label {
  font-size: 14px;
  color: #64748b;
  margin-bottom: 6px;
}

.text-input-area {
  width: 100%;
  max-width: 100%; /* 确保不超过父容器 */
  padding: 8px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  resize: vertical;
  font-size: 14px;
  transition: border-color 0.2s;
  box-sizing: border-box; /* 确保padding不会导致宽度超出 */
  min-height: 60px; /* 设置最小高度 */
  max-height: 120px; /* 设置最大高度 */
}

.text-input-area:focus {
  border-color: #3b82f6;
  outline: none;
}

.text-style-section {
  display: flex;
  flex-direction: column;
  gap: 12px;
  width: 100%;
}

.control-label {
  font-size: 14px;
  color: #64748b;
  margin-bottom: 6px;
}

.size-input-group {
  display: flex;
  align-items: center;
  gap: 6px;
}

.font-size {
  width: 60px;
  padding: 6px 8px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  font-size: 14px;
  transition: border-color 0.2s;
}

.font-size:focus {
  border-color: #3b82f6;
  outline: none;
}

.unit {
  color: #64748b;
  font-size: 14px;
}

.delete-message {
  display: flex;
  align-items: flex-start;
  gap: 12px;
  margin-bottom: 16px; /* 增加与按钮的间距 */
}

.warning-icon {
  display: flex;
  align-items: center;
  justify-content: center;
}

.message-title {
  font-weight: 600; /* 加粗标题 */
  margin-bottom: 4px;
  color: #334155;
  font-size: 15px; /* 略微增大字号 */
}

.message-desc {
  font-size: 13px;
  color: #64748b;
  line-height: 1.4; /* 增加行高提高可读性 */
}

#deletePointButton {
  background-color: #ef4444;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 10px 12px; /* 增加按钮高度 */
  font-size: 14px;
  font-weight: 600; /* 更加加粗文字 */
  cursor: pointer;
  width: 100%;
  transition: all 0.2s ease;
  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.2); /* 增强文字阴影 */
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1); /* 给按钮添加轻微阴影 */
  letter-spacing: 0.5px; /* 增加字间距 */
}

#deletePointButton:hover {
  background-color: #dc2626;
  transform: translateY(-1px); /* 悬停时轻微上移 */
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.15); /* 增强悬停时的阴影 */
}

#addCustomColorButton {
  background-color: #3b82f6;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 10px 12px; /* 增加按钮高度 */
  font-size: 14px;
  font-weight: 600; /* 更加加粗文字 */
  cursor: pointer;
  width: 100%;
  transition: all 0.2s ease;
  text-shadow: 0 1px 2px rgba(0, 0, 0, 0.2); /* 增强文字阴影 */
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1); /* 给按钮添加轻微阴影 */
  letter-spacing: 0.5px; /* 增加字间距 */
}

#addCustomColorButton:hover {
  background-color: #2563eb;
  transform: translateY(-1px); /* 悬停时轻微上移 */
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.15); /* 增强悬停时的阴影 */
}

.delete-confirm-content {
  border: 1px solid rgba(239, 68, 68, 0.2); /* 添加轻微的红色边框提示删除操作 */
}

.custom-color-input input[type="color"] {
  width: 32px;
  height: 32px;
  border: 1px solid #e2e8f0;
  padding: 0;
  background: none;
  cursor: pointer;
}

.custom-color-input input[type="text"] {
  flex: 1;
  padding: 6px 8px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  font-size: 14px;
  max-width: calc(100% - 40px); /* 确保不超出父容器，留出颜色选择器的空间 */
  box-sizing: border-box;
}

/* 悬停在路径点上时的样式效果 */
.waypoint {
  position: absolute;
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background-color: #3b82f6;
  cursor: pointer;
  transform-origin: center;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 12px;
  color: white;
  font-weight: bold;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
  border: 2px solid white;
  user-select: none;
}

.waypoint:hover {
  transform: scale(1.1);
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.3), 0 3px 6px rgba(0, 0, 0, 0.25);
  z-index: 1001;
}

.waypoint.selected {
  background-color: #2563eb;
  box-shadow: 0 0 0 4px rgba(59, 130, 246, 0.4), 0 4px 8px rgba(0, 0, 0, 0.3);
  transform: scale(1.2);
  z-index: 1002;
}

.selected-point-indicator {
  position: absolute;
  width: 30px;
  height: 30px;
  border-radius: 50%;
  border: 2px dashed #3b82f6;
  animation: pulse 1.5s infinite;
  opacity: 0.7;
  pointer-events: none;
}

@keyframes pulse {
  0% { transform: scale(1); opacity: 0.7; }
  50% { transform: scale(1.1); opacity: 0.5; }
  100% { transform: scale(1); opacity: 0.7; }
}

/* 悬浮菜单样式 */
.floating-menu {
  position: absolute;
  top: 20px;
  right: 20px;
  background-color: #ffffff;
  border-radius: 8px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  width: 260px;
  z-index: 1500;
  overflow: hidden;
  border: 1px solid #e2e8f0;
  transition: all 0.3s ease;
  max-height: calc(100% - 40px);
  overflow-y: auto;
}

.floating-menu-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  background-color: #f8fafc;
  border-bottom: 1px solid #e2e8f0;
  font-weight: 600;
  color: #334155;
  cursor: move;
  user-select: none;
  touch-action: none;
}

.header-title {
  display: flex;
  align-items: center;
  gap: 6px;
}

.header-icon {
  color: #3b82f6;
}

/* 拖动样式 */
.floating-menu.dragging {
  opacity: 0.9;
  box-shadow: 0 8px 30px rgba(0, 0, 0, 0.2);
}

.floating-menu-toggle {
  background: none;
  border: none;
  color: #64748b;
  cursor: pointer;
  padding: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 4px;
  transition: all 0.2s;
  width: 24px;
  height: 24px;
}

.floating-menu-toggle:hover {
  background-color: #e2e8f0;
  color: #334155;
}

.floating-menu-toggle svg {
  fill: currentColor;
  transition: transform 0.3s ease;
}

.floating-menu-toggle.collapsed svg {
  transform: rotate(-90deg);
}

.floating-menu-content {
  padding: 12px 16px;
  transform-origin: top center;
  transition: all 0.3s ease;
  max-height: 500px;
  opacity: 1;
}

.menu-section {
  margin-bottom: 16px;
  animation: fadeIn 0.3s ease-in-out;
}

@keyframes fadeIn {
  from { opacity: 0; transform: translateY(-10px); }
  to { opacity: 1; transform: translateY(0); }
}

.menu-section:last-child {
  margin-bottom: 0;
}

.section-title {
  font-size: 13px;
  font-weight: 600;
  color: #64748b;
  margin-bottom: 8px;
  display: flex;
  align-items: center;
}

.section-title::before {
  content: '';
  display: inline-block;
  width: 3px;
  height: 14px;
  background-color: #3b82f6;
  margin-right: 6px;
  border-radius: 3px;
}

.tool-button {
  display: flex;
  align-items: center;
  gap: 8px;
  width: 100%;
  padding: 8px 12px;
  background-color: #f8fafc;
  border: 1px solid #e2e8f0;
  border-radius: 6px;
  cursor: pointer;
  font-size: 14px;
  color: #334155;
  transition: all 0.2s ease;
  position: relative;
}

.tool-button:hover {
  background-color: #f1f5f9;
  transform: translateY(-1px);
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
}

.tool-button:active {
  transform: translateY(0);
  box-shadow: none;
}

.tool-button::after {
  content: attr(data-tooltip);
  position: absolute;
  bottom: 100%;
  left: 50%;
  transform: translateX(-50%);
  background-color: rgba(51, 65, 85, 0.9);
  color: white;
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 12px;
  white-space: nowrap;
  pointer-events: none;
  opacity: 0;
  transition: opacity 0.2s ease;
  z-index: 1600;
}

.tool-button:hover::after {
  opacity: 1;
}

.button-icon {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 16px;
  height: 16px;
  color: #64748b;
}

.tool-button:hover .button-icon {
  color: #3b82f6;
}

.buttons-row {
  display: flex;
  gap: 8px;
}

.boundary-button {
  flex: 1;
}

.input-row {
  display: flex;
  gap: 8px;
  margin-bottom: 8px;
}

.input-group {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.input-group label {
  font-size: 12px;
  color: #64748b;
}

.size-input {
  width: 100%;
  padding: 6px 8px;
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  font-size: 14px;
  background-color: #f8fafc;
  color: #334155;
  text-align: center;
}

.size-input:focus {
  outline: none;
  border-color: #3b82f6;
  background-color: #fff;
}

.apply-button {
  background-color: #3b82f6;
  color: white;
  border: none;
  font-weight: 500;
  justify-content: center;
  margin-top: 4px;
}

.apply-button:hover {
  background-color: #2563eb;
}

.apply-button .button-icon {
  color: white;
}
</style>
