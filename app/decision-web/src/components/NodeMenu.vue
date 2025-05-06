<template>
  <div class="edit-menu" :class="{ 'open': visible }">
    <div class="edit-menu-content" v-if="selectedNode">
      <!-- 顶部导航栏和标题 -->
      <div class="menu-header">
        <h3>编辑节点</h3>
        <button class="close-button" @click="close" title="关闭">×</button>
      </div>

      <!-- 标识信息区域 -->
      <div class="info-area">
        <span class="node-id">节点 #{{ selectedNode.id.value }}</span>
        <span class="node-title">{{ selectedNode.text.content }}</span>
      </div>

      <!-- 主要编辑区域 -->
      <div class="edit-area">
        <!-- 主要设置选项卡 -->
        <div class="tab-container">
          <div class="tab-headers">
            <button
              v-for="(tab, index) in tabs"
              :key="index"
              :class="['tab-button', { active: activeTab === index }]"
              @click="activeTab = index"
            >
              {{ tab.label }}
            </button>
          </div>

          <!-- 内容选项卡 -->
          <div class="tab-content">
            <!-- 外观选项卡 -->
            <div v-if="activeTab === 0" class="tab-pane appearance-tab">
              <div class="setting-card">
                <label for="node-color">颜色</label>
                <div class="color-control">
                  <input type="color" id="node-color" v-model="selectedNode.color.fillColor" />
                  <span class="color-code">{{ selectedNode.color.fillColor }}</span>
                </div>
              </div>

              <div class="setting-card size-controls">
                <label for="node-size">大小</label>
                <div class="size-inputs">
                  <div class="input-with-label">
                    <input type="number" id="node-width" v-model.number="selectedNode.shape.width" min="40" max="400" @input="updateSize" />
                    <span>宽</span>
                  </div>
                  <div class="size-separator">×</div>
                  <div class="input-with-label">
                    <input type="number" id="node-height" v-model.number="selectedNode.shape.height" min="30" max="300" @input="updateSize" />
                    <span>高</span>
                  </div>
                </div>
              </div>

              <div class="setting-card">
                <label for="node-text-content">文本</label>
                <input type="text" id="node-text-content" v-model="selectedNode.text.content" placeholder="节点文本内容" />
              </div>

              <div class="setting-card">
                <label for="text-color">文本颜色</label>
                <div class="color-control">
                  <input type="color" id="text-color" v-model="selectedNode.text.color" />
                  <span class="color-code">{{ selectedNode.text.color }}</span>
                </div>
              </div>

              <div class="setting-card">
                <label for="text-size">文本大小</label>
                <div class="slider-control">
                  <input
                    type="range"
                    id="text-size"
                    v-model.number="selectedNode.text.size"
                    min="10"
                    max="32"
                    step="1"
                  />
                  <span class="slider-value">{{ selectedNode.text.size }}px</span>
                </div>
              </div>
            </div>

            <!-- 图标选项卡 -->
            <div v-if="activeTab === 1" class="tab-pane icon-tab">
              <div class="setting-card" v-if="selectedNode">
                <label for="icon-svg">SVG路径</label>
                <textarea
                  id="icon-svg"
                  v-model="iconSvgPath"
                  placeholder="输入SVG路径代码"
                  rows="3"
                  @input="updateIcon"
                ></textarea>
              </div>

              <div class="setting-card icon-preview-row" v-if="selectedNode.icon?.svgPath">
                <div class="icon-display" :style="{ backgroundColor: selectedNode.icon?.bgColor }">
                  <svg viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                    <path :d="selectedNode.icon?.svgPath" fill="#fff" />
                  </svg>
                </div>
                <div class="icon-actions">
                  <span>图标预览</span>
                  <div class="color-control">
                    <input type="color" v-model="iconColor" @input="updateIconColor" />
                  </div>
                </div>
              </div>
            </div>

            <!-- 属性选项卡 -->
            <div v-if="activeTab === 2" class="tab-pane properties-tab">
              <div class="setting-card radio-row" v-if="selectedNode">
                <label>节点类型</label>
                <div class="radio-buttons">
                  <label v-if="selectedNode.taskConfig.nodeType !== 'root'"
                  class="radio-button" :class="{ active: selectedNode.taskConfig.nodeType === 'task' }">
                    <input type="radio" v-model="selectedNode.taskConfig.nodeType" value="task" @change="updateNodeType" />
                    <span>任务</span>
                  </label>
                  <label v-if="selectedNode.taskConfig.nodeType !== 'root'"
                  class="radio-button" :class="{ active: selectedNode.taskConfig.nodeType === 'branch' }">
                    <input type="radio" v-model="selectedNode.taskConfig.nodeType" value="branch" @change="updateNodeType" />
                    <span>分支</span>
                  </label>
                  <label v-if="selectedNode.taskConfig.nodeType == 'root'"
                  class="radio-button" :class="{ active: selectedNode.taskConfig.nodeType === 'root' }
                  ">
                    <input type="radio" v-model="selectedNode.taskConfig.nodeType" value="root" @change="updateNodeType" />
                    <span>根节点</span>
                  </label>
                </div>
              </div>

              <div class="setting-card radio-row" v-if="selectedNode.taskConfig.nodeType === 'task'">
                <label>执行模式</label>
                <div class="radio-buttons">
                  <label class="radio-button" :class="{ active: selectedNode.taskConfig.mode === Mode.Move }">
                    <input type="radio" v-model="selectedNode.taskConfig.mode" value='Move' @change="updateNodeMode" />
                    <span>导航模式</span>
                  </label>
                  <label class="radio-button" :class="{ active: selectedNode.taskConfig.mode === Mode.Stay }">
                    <input type="radio" v-model="selectedNode.taskConfig.mode" value='Stay' @change="updateNodeMode" />
                    <span>原地停留</span>
                  </label>
                  <label class="radio-button" :class="{ active: selectedNode.taskConfig.mode === Mode.Limit }">
                    <input type="radio" v-model="selectedNode.taskConfig.mode" value='Limit' @change="updateNodeMode" />
                    <span>限制速度</span>
                  </label>
                </div>
              </div>

              <div class="setting-card" v-if="selectedNode.taskConfig.nodeType === 'task'">
                <label for="reset-time">重置时间</label>
                <div class="slider-control">
                  <input type="range" id="reset-time" v-model.number="selectedNode.taskConfig.resetTime" min="0" max="100" @input="updateResetTime" />
                  <span class="slider-value">{{ selectedNode.taskConfig.resetTime ? selectedNode.taskConfig.resetTime : 0 }}秒</span>
                </div>
              </div>

              <div class="setting-card" v-if="selectedNode.taskConfig.mode === Mode.Move">
                <label for="waypoint-selector">任务航点</label>
                <select id="waypoint-selector" v-model="selectedNode.taskConfig.waypointId" @change="updateWaypoint">
                  <option value="">无</option>
                  <option v-for="waypoint in waypoints" :key="waypoint.id.value" :value="waypoint.id.value">
                    {{ waypoint.text.value }}
                  </option>
                </select>
              </div>

              <div class="setting-card" v-if="selectedNode.taskConfig.mode === Mode.Limit">
                <label for="limit-spin-speed">最大旋转速度</label>
                <div class="slider-control">
                  <input type="range" id="limit-spin-speed" v-model.number="selectedNode.taskConfig.spin" min="0" max="10" step="0.1"/>
                  <span class="slider-value">{{ (selectedNode.taskConfig.spin ? selectedNode.taskConfig.spin : 0).toFixed(1) }} 弧度/秒</span>
                </div>
              </div>

              <div class="setting-card" v-if="selectedNode.taskConfig.mode === Mode.Limit">
                <label for="limit-linear-speed">最大线速度</label>
                <div class="slider-control">
                  <input type="range" id="limit-linear-speed" v-model.number="selectedNode.taskConfig.linear" min="0" max="10" step="0.1"/>
                  <span class="slider-value">{{ (selectedNode.taskConfig.linear ? selectedNode.taskConfig.linear : 0).toFixed(1) }} 米/秒</span>
                </div>
              </div>

            </div>

            <!-- 优先级选项卡 -->
            <div v-if="activeTab === 3" class="tab-pane edges-tab">
              <div class="setting-card">
                <div class="edges-header">
                  <label>优先级列表</label>
                  <span class="edges-count" v-if="nodeEdges.length">共 {{ nodeEdges.length }} 个连接</span>
                </div>

                <div class="edges-empty" v-if="!nodeEdges.length">
                  <span>当前节点没有连接的边</span>
                </div>

                <draggable
                  v-model="nodeEdges"
                  item-key="id"
                  class="edges-list"
                  handle=".edge-drag-handle"
                  @end="handleEdgeOrderChanged"
                >
                  <template #item="{element, index}">
                    <div class="edge-item">
                      <div class="edge-drag-handle">
                        <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                          <circle cx="8" cy="6" r="1"/>
                          <circle cx="8" cy="12" r="1"/>
                          <circle cx="8" cy="18" r="1"/>
                          <circle cx="16" cy="6" r="1"/>
                          <circle cx="16" cy="12" r="1"/>
                          <circle cx="16" cy="18" r="1"/>
                        </svg>
                      </div>
                      <div class="edge-info">
                        <div class="edge-primary">
                          <span class="edge-name">{{ getEdgeTargetName(element) }}</span>
                          <span class="edge-id">连接线 ID:  #{{ element.id.value }} {{ element.config.label ? `| 说明:${element.config.label}` : '' }}</span>
                        </div>
                        <div class="edge-priority">
                          <span class="priority-label">优先级</span>
                          <span class="priority-value">{{ index + 1 }}</span>
                        </div>
                      </div>
                    </div>
                  </template>
                </draggable>
              </div>

              <div class="setting-card" v-if="nodeEdges.length">
                <div class="edge-priority-info">
                  <div class="info-icon">
                    <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                      <circle cx="12" cy="12" r="10"/>
                      <line x1="12" y1="16" x2="12" y2="12"/>
                      <line x1="12" y1="8" x2="12" y2="8"/>
                    </svg>
                  </div>
                  <span>拖动列表项可调整边缘的优先级顺序。列表顶部的边具有最高优先级。</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, watch } from 'vue';
import { Node } from '@/types/Node';
import { nodes as nodeList, points as waypoints, showError } from '@/types/Manger';
import { Mode, NodeType } from '@/types/NodeBase';
import { Edge } from '@/types/EdgeBase';
import draggable from 'vuedraggable';

// 属性定义
interface Props {
  visible: boolean;
  selectedNodeId?: number | null;
}

const props = withDefaults(defineProps<Props>(), {
  visible: false,
  selectedNodeId: null
});

const emit = defineEmits(['update:visible', 'node-updated', 'close']);

// 响应式状态
const selectedNode = computed<Node | null>(() => {
  if (props.selectedNodeId === null) return null;
  return nodeList.value.find(node => node.id.value === props.selectedNodeId) || null;
});

// 节点的边列表
const nodeEdges = ref<Edge[]>([]);

// 相关表单数据
const iconSvgPath = ref('');
const iconColor = ref('#3B82F6');
// const spinSpeed = ref(0);
// const linear = ref(0);
// 选项卡控制
const activeTab = ref(0);
const tabs = ref<{ label: string }[]>([]);
const AllTabs = [
  { label: '外观' },
  { label: '图标' },
  { label: '属性' },
  { label: '优先级' }
];


// 监听选中节点变化
watch(() => selectedNode.value, (newNode) => {
  if (newNode) {
    // 初始化表单数据
    activeTab.value = 0;
    iconSvgPath.value = newNode.icon?.svgPath || '';
    iconColor.value = newNode.icon?.bgColor || '#3B82F6';
    // 初始化边缘数据
    if (newNode.edges && newNode.edges.length > 0) {
      nodeEdges.value = newNode.edges;
      console.log(newNode.edges);
    } else {
      nodeEdges.value = [];
    }
    if (newNode.taskConfig.nodeType === NodeType.Task) {
      tabs.value = AllTabs.slice(0, 3);
    } else {
      tabs.value = AllTabs;
    }
  }
}, { immediate: true });

// 方法
const close = () => {
  emit('update:visible', false);
  emit('close');
};

const updateSize = () => {
  if (!selectedNode.value) return;

  // 确保宽高保持合理比例
  const minWidth = 40;
  const minHeight = 30;

  if (selectedNode.value.shape.width < minWidth) {
    selectedNode.value.shape.width = minWidth;
  }

  if (selectedNode.value.shape.height < minHeight) {
    selectedNode.value.shape.height = minHeight;
  }
};

const updateIcon = () => {
  if (!selectedNode.value) return;

  if (!selectedNode.value.icon) {
    selectedNode.value.icon = {
      svgPath: iconSvgPath.value,
      bgColor: iconColor.value
    };
  } else {
    selectedNode.value.icon.svgPath = iconSvgPath.value;
  }
};

const updateIconColor = () => {
  if (!selectedNode.value || !selectedNode.value.icon) return;
  selectedNode.value.icon.bgColor = iconColor.value;
};

const updateNodeType = () => {
  if (!selectedNode.value) return;
  if (selectedNode.value.taskConfig.nodeType === 'task') {
    if (selectedNode.value.edges.length > 0) {
      selectedNode.value.taskConfig.nodeType = NodeType.Branch;
      showError('任务节点不能有连接的边');
    }
  }

  if (selectedNode.value.taskConfig.nodeType === 'root') {
    selectedNode.value.taskConfig.mode = Mode.Stay;
    selectedNode.value.taskConfig.waypointId = null;
    selectedNode.value.taskConfig.spin = 0;
    selectedNode.value.taskConfig.resetTime = 0;
  }
};

const updateNodeMode = () => {
  // if (!selectedNode.value) return;
  // selectedNode.value.taskConfig.mode = mode;
};

const updateResetTime = () => {
  // if (!selectedNode.value) return;
  // selectedNode.value.taskConfig.resetTime = resetTime;
};

const updateWaypoint = () => {
  // if (!selectedNode.value) return;
  // selectedNode.value.taskConfig.waypointId = waypointId;
};

// const updateSpinSpeed = () => {
//   if (!selectedNode.value) return;
//   selectedNode.value.taskConfig.spin = spinSpeed.value / 10;
// };

// 获取边的目标节点名称
const getEdgeTargetName = (edge: Edge): string => {
  const targetNode = nodeList.value.find(node => node.id.value === edge.targetId);
  return targetNode ? targetNode.text.content : `未知节点 #${edge.targetId}`;
};

// 处理边顺序变化
const handleEdgeOrderChanged = () => {
  if (!selectedNode.value) return;

  // 更新节点的边顺序
  if (nodeEdges.value.length > 0) {
    selectedNode.value.edges = [...nodeEdges.value];
    emit('node-updated', selectedNode.value);
  }
};

</script>

<style scoped>
/* 整体布局 */
.edit-menu {
  position: fixed;
  top: 0;
  right: -450px;
  width: 450px;
  height: 100vh;
  z-index: 1000;
  transition: right 0.35s cubic-bezier(0.16, 1, 0.3, 1);
  overflow: hidden;
  background-color: #f8f9fa;
  border-left: 1px solid #e9ecef;
  box-shadow: -6px 0 35px rgba(0, 0, 0, 0.07);
  color: #212529;
  display: flex;
}

.edit-menu.open {
  right: 0;
}

.edit-menu-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

/* 头部样式 */
.menu-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 22px 28px;
  border-bottom: 1px solid #e9ecef;
}

.menu-header h3 {
  margin: 0;
  font-size: 19px;
  font-weight: 600;
  color: #212529;
}

.close-button {
  width: 36px;
  height: 36px;
  display: flex;
  align-items: center;
  justify-content: center;
  border: none;
  background: #e9ecef;
  color: #495057;
  font-size: 22px;
  border-radius: 50%;
  cursor: pointer;
  transition: all 0.2s ease;
}

.close-button:hover {
  background-color: #dee2e6;
  color: #000;
  transform: scale(1.05);
}

/* 信息区域 */
.info-area {
  padding: 18px 28px;
  border-bottom: 1px solid #e9ecef;
  background-color: #fff;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.03);
  margin-bottom: 10px;
}

.node-id {
  font-size: 13px;
  color: #6c757d;
  display: block;
  margin-bottom: 6px;
}

.node-title {
  font-size: 26px;
  font-weight: 600;
  color: #212529;
  display: block;
}

/* 编辑区域 */
.edit-area {
  flex: 1;
  overflow-y: auto;
  padding: 18px 28px 28px 28px;
}

.edit-area::-webkit-scrollbar {
  width: 6px;
}

.edit-area::-webkit-scrollbar-track {
  background: transparent;
}

.edit-area::-webkit-scrollbar-thumb {
  background-color: #ced4da;
  border-radius: 3px;
}

/* 选项卡容器 */
.tab-container {
  display: flex;
  flex-direction: column;
}

.tab-headers {
  display: flex;
  margin-bottom: 25px;
  background-color: #e9ecef;
  border-radius: 10px;
  padding: 5px;
}

.tab-button {
  flex: 1;
  padding: 12px 15px;
  background: transparent;
  border: none;
  cursor: pointer;
  color: #495057;
  font-size: 14px;
  font-weight: 500;
  transition: all 0.25s ease;
  border-radius: 8px;
  text-align: center;
}

.tab-button::after {
  display: none;
}

.tab-button.active {
  background-color: #fff;
  color: #3B82F6;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
}

.tab-button:not(.active):hover {
  background-color: #dee2e6;
}

.tab-content {
  display: flex;
  flex-direction: column;
  gap: 20px;
}

.tab-pane {
  display: flex;
  flex-direction: column;
  gap: 20px;
}

/* 设置卡片 (替代 .setting-row) */
.setting-card {
  background-color: #fff;
  border-radius: 12px;
  padding: 20px;
  box-shadow: 0 3px 10px rgba(0, 0, 0, 0.05);
  transition: box-shadow 0.2s ease;
  box-sizing: border-box;
  width: 100%;
  overflow: hidden;
}

.setting-card:hover {
  box-shadow: 0 5px 15px rgba(0, 0, 0, 0.07);
}

.setting-card label {
  display: block;
  font-size: 14px;
  font-weight: 500;
  color: #495057;
  margin-bottom: 12px;
}

.setting-card *,
.setting-card *::before,
.setting-card *::after {
  box-sizing: border-box;
}

/* 输入样式 */
input[type="text"],
input[type="number"],
textarea,
select {
  padding: 14px 16px;
  background-color: #f8f9fa;
  border: 1px solid #dee2e6;
  border-radius: 8px;
  font-size: 14px;
  width: 100%;
  color: #212529;
  transition: all 0.2s ease;
  box-sizing: border-box;
  max-width: 100%;
}

input[type="text"]:focus,
input[type="number"]:focus,
textarea:focus,
select:focus {
  outline: none;
  background-color: white;
  border-color: #80bdff;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.15);
}

/* 颜色控制 */
.color-control {
  display: flex;
  align-items: center;
  gap: 15px;
  flex-wrap: wrap;
  width: 100%;
}

input[type="color"] {
  width: 44px;
  height: 44px;
  border: 1px solid #dee2e6;
  border-radius: 10px;
  cursor: pointer;
  background: none;
  padding: 3px;
  overflow: hidden;
  transition: transform 0.2s ease;
  flex-shrink: 0;
}
input[type="color"]::-webkit-color-swatch {
  border: none;
  border-radius: 7px;
}
input[type="color"]::-moz-color-swatch {
  border: none;
  border-radius: 7px;
}
input[type="color"]:hover {
  transform: scale(1.05);
}

.color-code {
  font-family: 'Fira Code', monospace;
  font-size: 14px;
  color: #495057;
  background-color: #e9ecef;
  padding: 8px 12px;
  border-radius: 6px;
  text-transform: uppercase;
  flex: 1;
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

/* 滑块控制 */
.slider-control {
  display: flex;
  align-items: center;
  gap: 18px;
  width: 100%;
}

input[type="range"] {
  flex: 1;
  height: 8px;
  -webkit-appearance: none;
  appearance: none;
  background-color: #e9ecef;
  border-radius: 4px;
  cursor: pointer;
  min-width: 0;
}

input[type="range"]::-webkit-slider-thumb {
  -webkit-appearance: none;
  width: 20px;
  height: 20px;
  background-color: #3B82F6;
  border-radius: 50%;
  cursor: pointer;
  border: 3px solid white;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.15);
  transition: transform 0.15s ease;
}
input[type="range"]:active::-webkit-slider-thumb {
  transform: scale(1.1);
}

.slider-value {
  min-width: 60px;
  font-family: 'Fira Code', monospace;
  font-size: 14px;
  color: #495057;
  background-color: #e9ecef;
  padding: 8px 12px;
  border-radius: 6px;
  text-align: center;
  flex-shrink: 0;
}

/* 尺寸控制 */
.size-inputs {
  display: flex;
  align-items: center;
  gap: 12px;
  width: 100%;
}

.input-with-label {
  flex: 1;
  display: flex;
  align-items: center;
  background-color: #f8f9fa;
  border: 1px solid #dee2e6;
  border-radius: 8px;
  overflow: hidden;
  min-width: 0;
}

.input-with-label input {
  width: 75px;
  text-align: center;
  padding: 12px;
  border: none;
  background: transparent;
  border-right: 1px solid #dee2e6;
  flex-shrink: 1;
  min-width: 0;
}
.input-with-label input:focus {
  box-shadow: none;
  background: white;
}

.input-with-label span {
  padding: 0 15px;
  color: #6c757d;
  font-size: 13px;
  white-space: nowrap;
}

.size-separator {
  color: #adb5bd;
  font-size: 18px;
  font-weight: 400;
  flex-shrink: 0;
}

/* 单选按钮样式 */
.radio-buttons {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
  width: 100%;
}

.radio-button {
  display: flex;
  align-items: center;
  padding: 12px 20px;
  background-color: #f8f9fa;
  border: 1px solid #dee2e6;
  border-radius: 10px;
  cursor: pointer;
  transition: all 0.2s ease;
  flex: 1;
  min-width: 100px;
  justify-content: center;
}

.radio-button.active {
  background-color: #e7f1ff;
  border-color: #a5c8ff;
  color: #0d6efd;
  box-shadow: 0 2px 4px rgba(59, 130, 246, 0.1);
}

.radio-button:hover:not(.active) {
  background-color: #e9ecef;
  border-color: #ced4da;
}

.radio-button input {
  display: none;
}

.radio-button span {
  font-size: 14px;
  font-weight: 500;
  white-space: nowrap;
}

/* 图标预览 */
.icon-preview-row {
  display: flex;
  align-items: center;
  gap: 20px;
  flex-wrap: wrap;
}

.icon-display {
  width: 70px;
  height: 70px;
  border-radius: 12px;
  background-color: #3B82F6;
  display: flex;
  align-items: center;
  justify-content: center;
  overflow: hidden;
  border: 1px solid rgba(0,0,0,0.05);
  box-shadow: 0 2px 5px rgba(0,0,0,0.1);
  flex-shrink: 0;
}

.icon-display svg {
  width: 36px;
  height: 36px;
}

.icon-actions {
  display: flex;
  flex-direction: column;
  gap: 12px;
  flex: 1;
  min-width: 0;
}

.icon-actions > span {
  font-size: 14px;
  font-weight: 500;
  color: #495057;
}

/* 底部操作区域 */
.menu-footer {
  display: flex;
  justify-content: flex-end;
  padding: 22px 28px;
  border-top: 1px solid #e9ecef;
  background-color: #fff;
}

.save-button {
  background-color: #3B82F6;
  color: white;
  border: none;
  padding: 14px 28px;
  border-radius: 10px;
  font-size: 15px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.25s ease;
  box-shadow: 0 3px 8px rgba(59, 130, 246, 0.2);
}

.save-button:hover {
  background-color: #2563EB;
  transform: translateY(-2px);
  box-shadow: 0 6px 14px rgba(59, 130, 246, 0.3);
}

.save-button:active {
  transform: translateY(0);
  background-color: #1d4ed8;
  box-shadow: 0 2px 5px rgba(59, 130, 246, 0.2);
}

/* 边缘选项卡样式 */
.edges-tab .edges-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 15px;
}

.edges-count {
  font-size: 13px;
  color: #6c757d;
  background-color: #e9ecef;
  padding: 5px 10px;
  border-radius: 6px;
}

.edges-empty {
  text-align: center;
  padding: 25px 0;
  color: #6c757d;
  font-style: italic;
  background-color: #f8f9fa;
  border-radius: 8px;
  border: 1px dashed #dee2e6;
}

.edges-list {
  display: flex;
  flex-direction: column;
  gap: 10px;
  margin-top: 12px;
  max-height: 350px;
  overflow-y: auto;
}

.edge-item {
  display: flex;
  align-items: center;
  background-color: #f8f9fa;
  border: 1px solid #dee2e6;
  border-radius: 10px;
  overflow: hidden;
  transition: all 0.2s ease;
}

.edge-item:hover {
  border-color: #ced4da;
  background-color: #f1f3f5;
  transform: translateY(-2px);
  box-shadow: 0 3px 8px rgba(0, 0, 0, 0.05);
}

.edge-drag-handle {
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 0 15px;
  height: 100%;
  color: #adb5bd;
  cursor: grab;
  background-color: #f1f3f5;
  border-right: 1px solid #dee2e6;
}

.edge-drag-handle:active {
  cursor: grabbing;
}

.edge-info {
  flex: 1;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 15px;
}

.edge-primary {
  display: flex;
  flex-direction: column;
  gap: 3px;
}

.edge-name {
  font-size: 15px;
  font-weight: 500;
  color: #212529;
}

.edge-id {
  font-size: 12px;
  color: #6c757d;
}

.edge-priority {
  display: flex;
  flex-direction: column;
  align-items: center;
  background-color: #e7f1ff;
  padding: 8px 12px;
  border-radius: 8px;
}

.priority-label {
  font-size: 10px;
  color: #4dabf7;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.priority-value {
  font-size: 16px;
  font-weight: 600;
  color: #1971c2;
}

.edge-priority-info {
  display: flex;
  align-items: center;
  gap: 15px;
  color: #6c757d;
  background-color: #fff8e5;
  padding: 15px;
  border-radius: 10px;
  border: 1px solid #ffe8cc;
}

.info-icon {
  color: #ff922b;
  flex-shrink: 0;
}

/* 支持移动设备 */
@media (max-width: 640px) {
  .edit-menu {
    width: 100%;
    right: -100%;
    box-shadow: none;
    border-left: none;
  }
  .tab-button {
    padding: 10px 12px;
    font-size: 13px;
  }
  .setting-card {
    padding: 15px;
  }
  .info-area, .menu-header, .menu-footer {
    padding: 15px 20px;
  }

  .color-control, .slider-control {
    gap: 10px;
  }

  .radio-button {
    min-width: 80px;
    padding: 10px 15px;
  }

  .edge-info {
    padding: 12px;
  }

  .edge-priority {
    padding: 6px 10px;
  }

  .edge-priority-info {
    padding: 12px;
    font-size: 13px;
  }
}
</style>
