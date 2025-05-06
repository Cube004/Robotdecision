<template>
  <div class="edit-menu" :class="{ 'open': visible }">
    <div class="edit-menu-content" v-if="selectedEdge">
      <!-- 顶部导航栏和标题 -->
      <div class="menu-header">
        <h3>编辑连线</h3>
        <button class="close-button" @click="close" title="关闭">×</button>
      </div>

      <!-- 标识信息区域 -->
      <div class="info-area">
        <span class="edge-id">连线 #{{ selectedEdge.id.value }}</span>
        <span class="edge-title">{{ selectedEdge.config.label || '无标签' }}</span>
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
            <!-- 基本属性选项卡 -->
            <div v-if="activeTab === 0" class="tab-pane appearance-tab">
              <div class="setting-card">
                <label for="edge-color">线条颜色</label>
                <div class="color-control">
                  <input type="color" id="edge-color" v-model="selectedEdge.config.color" />
                  <span class="color-code">{{ selectedEdge.config.color }}</span>
                </div>
              </div>

              <div class="setting-card">
                <label>线条类型</label>
                <div class="radio-buttons">
                  <label class="radio-button" :class="{ active: selectedEdge.config.type === 'straight' }">
                    <input type="radio" v-model="selectedEdge.config.type" value="straight" />
                    <span>直线</span>
                  </label>
                  <label class="radio-button" :class="{ active: selectedEdge.config.type === 'curve' }">
                    <input type="radio" v-model="selectedEdge.config.type" value="curve" />
                    <span>曲线</span>
                  </label>
                </div>
              </div>

              <div class="setting-card" v-if="selectedEdge.config.type === 'curve'">
                <label for="edge-curvature">曲率</label>
                <div class="slider-control">
                  <input
                    type="range"
                    id="edge-curvature"
                    v-model.number="selectedEdge.config.curvature"
                    min="0"
                    max="1"
                    step="0.1"
                  />
                  <span class="slider-value">{{ selectedEdge.config.curvature?.toFixed(1) || 0 }}</span>
                </div>
              </div>

              <div class="setting-card">
                <label for="edge-width">线条宽度</label>
                <div class="slider-control">
                  <input
                    type="range"
                    id="edge-width"
                    v-model.number="selectedEdge.config.width"
                    min="1"
                    max="10"
                    step="0.5"
                  />
                  <span class="slider-value">{{ selectedEdge.config.width || 1 }}px</span>
                </div>
              </div>

              <div class="setting-card">
                <label for="edge-label">连线标签</label>
                <input type="text" id="edge-label" v-model="selectedEdge.config.label" placeholder="输入连线标签" />
              </div>

              <!-- <div class="setting-card">
                <label>虚线效果</label>
                <div class="toggle-switch">
                  <input type="checkbox" id="dashed-toggle" v-model="selectedEdge.config.dashed" />
                  <label for="dashed-toggle" class="toggle-label">
                    <span class="toggle-inner"></span>
                    <span class="toggle-switch-label">{{ selectedEdge.config.dashed ? '开启' : '关闭' }}</span>
                  </label>
                </div>
              </div> -->
            </div>

            <!-- 条件编辑器选项卡 -->
            <div v-if="activeTab === 1" class="tab-pane conditions-tab">
              <div v-for="(condition, index) in selectedEdge.conditions" :key="index" class="condition-card setting-card">
                <div class="condition-header">
                  <h4>条件 #{{ index + 1 }}</h4>
                  <button class="delete-condition-btn" @click="removeCondition(index)" title="删除条件">
                    <span>×</span>
                  </button>
                </div>

                <div class="condition-body">
                  <div class="condition-setting">
                    <label for="datetype">数据类型</label>
                    <select id="datetype" v-model="condition.datetype">
                      <option v-for="(option, idx) in dataTypeOptions" :key="idx" :value="option.value">
                        {{ option.label }}
                      </option>
                    </select>
                  </div>

                  <div class="condition-setting">
                    <label for="metricType">度量类型</label>
                    <select id="metricType" v-model="condition.metricType">
                      <option v-for="(option, idx) in metricTypeOptions" :key="idx" :value="option.value">
                        {{ option.label }}
                      </option>
                    </select>
                  </div>

                  <div class="condition-setting">
                    <label>时间范围类型</label>
                    <div class="radio-buttons">
                      <label
                        v-for="(option, idx) in temporalScopeOptions"
                        :key="idx"
                        class="radio-button"
                        :class="{ active: condition.temporalScope.type === option.value }"
                      >
                        <input
                          type="radio"
                          :name="`temporal-scope-${index}`"
                          v-model="condition.temporalScope.type"
                          :value="option.value"
                        />
                        <span>{{ option.label }}</span>
                      </label>
                    </div>
                  </div>

                  <div class="condition-setting" v-if="condition.temporalScope.type === 3">
                    <label for="rollingWindow">滚动窗口</label>
                    <div class="slider-control">
                      <input
                        type="range"
                        :id="`rollingWindow-${index}`"
                        v-model.number="condition.temporalScope.rollingWindow"
                        min="1"
                        max="60"
                        step="1"
                      />
                      <span class="slider-value">{{ condition.temporalScope.rollingWindow || 1 }}分钟</span>
                    </div>
                  </div>

                  <div class="condition-setting minmax-row">
                    <div class="input-group">
                      <label for="min">最小值</label>
                      <input type="number" :id="`min-${index}`" v-model.number="condition.min" placeholder="最小值" />
                    </div>
                    <div class="input-group">
                      <label for="max">最大值</label>
                      <input type="number" :id="`max-${index}`" v-model.number="condition.max" placeholder="最大值" />
                    </div>
                  </div>
                </div>
              </div>

              <div class="add-condition-card setting-card">
                <button class="add-condition-btn" @click="addCondition">
                  <span class="add-icon">+</span>
                  <span>添加条件</span>
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>

      <!-- 底部操作区域 -->
      <!-- <div class="menu-footer">
        <button class="save-button" @click="saveChanges">保存更改</button>
      </div> -->
    </div>
  </div>
</template>

<script setup lang="ts" name="EdgeMenu">
import { ref, computed } from 'vue';
import type { Edge } from '@/types/EdgeBase';
import type { Condition } from '@/types/Condition';
import { edges } from '@/types/Manger';

// 属性定义
interface Props {
  visible: boolean;
  selectedEdgeId?: number | null;
}

const props = withDefaults(defineProps<Props>(), {
  visible: false,
  selectedEdgeId: null
});

const emit = defineEmits(['update:visible', 'edge-updated', 'close']);

// 使用全局边数据
// const edgeList = ref<Edge[]>([]);

// 响应式状态
const selectedEdge = computed<Edge | null>(() => {
  if (!props.selectedEdgeId) return null;
  return edges.value.find(edge => edge.id.value === props.selectedEdgeId) || null;
});

// 选项卡控制
const activeTab = ref(0);
const tabs = [
  { label: '基本属性' },
  { label: '条件编辑' }
];

// 数据类型选项
const dataTypeOptions = [
  { value: 'temperature', label: '温度' },
  { value: 'humidity', label: '湿度' },
  { value: 'pressure', label: '气压' },
  { value: 'speed', label: '速度' },
  { value: 'battery', label: '电量' }
];

// 度量类型选项
const metricTypeOptions = [
  { value: 1, label: '瞬时值' },
  { value: 2, label: '平均值' },
  { value: 3, label: '最大值' },
  { value: 4, label: '最小值' }
];

// 时间范围类型选项
const temporalScopeOptions = [
  { value: 1, label: '实时' },
  { value: 2, label: '累计' },
  { value: 3, label: '滚动窗口' }
];

// 方法
const close = () => {
  emit('update:visible', false);
  emit('close');
};

const addCondition = () => {
  if (!selectedEdge.value) return;

  // 创建新的空白条件
  const newCondition: Condition = {
    datetype: 'temperature',
    metricType: 1,
    temporalScope: {
      type: 1,
      rollingWindow: null
    },
    min: 0,
    max: 100
  };

  selectedEdge.value.conditions.push(newCondition);
};

const removeCondition = (index: number) => {
  if (!selectedEdge.value) return;
  selectedEdge.value.conditions.splice(index, 1);
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

.edge-id {
  font-size: 13px;
  color: #6c757d;
  display: block;
  margin-bottom: 6px;
}

.edge-title {
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

/* 设置卡片 */
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

/* 切换开关样式 */
.toggle-switch {
  position: relative;
  display: inline-block;
  margin-top: 5px;
}

.toggle-switch input {
  opacity: 0;
  width: 0;
  height: 0;
}

.toggle-label {
  display: flex;
  align-items: center;
  cursor: pointer;
}

.toggle-inner {
  position: relative;
  display: inline-block;
  width: 50px;
  height: 26px;
  background-color: #e9ecef;
  border-radius: 15px;
  margin-right: 12px;
  transition: all 0.3s;
}

.toggle-inner:before {
  content: '';
  position: absolute;
  width: 20px;
  height: 20px;
  left: 3px;
  bottom: 3px;
  background-color: white;
  border-radius: 50%;
  transition: 0.3s;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
}

input:checked + .toggle-label .toggle-inner {
  background-color: #3B82F6;
}

input:checked + .toggle-label .toggle-inner:before {
  transform: translateX(24px);
}

.toggle-switch-label {
  font-size: 14px;
  font-weight: 500;
  color: #495057;
}

/* 条件卡片样式 */
.condition-card {
  border: 1px solid #e9ecef;
  margin-bottom: 5px;
}

.condition-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
  padding-bottom: 12px;
  border-bottom: 1px solid #e9ecef;
}

.condition-header h4 {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
  color: #3B82F6;
}

.delete-condition-btn {
  width: 28px;
  height: 28px;
  display: flex;
  align-items: center;
  justify-content: center;
  border: none;
  background: #f2f4f6;
  color: #dc3545;
  font-size: 18px;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s;
}

.delete-condition-btn:hover {
  background-color: #fbe9e7;
  color: #d32f2f;
}

.condition-body {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.condition-setting {
  margin-bottom: 5px;
}

.minmax-row {
  display: flex;
  gap: 15px;
}

.input-group {
  flex: 1;
}

/* 添加条件按钮 */
.add-condition-card {
  display: flex;
  justify-content: center;
  padding: 15px;
  background-color: #fff;
  border: 1px dashed #ced4da;
}

.add-condition-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 10px;
  background-color: #e7f1ff;
  color: #3B82F6;
  border: none;
  padding: 12px 24px;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s;
}

.add-condition-btn:hover {
  background-color: #d3e5ff;
  transform: translateY(-1px);
}

.add-icon {
  font-size: 18px;
  font-weight: 500;
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

  .minmax-row {
    flex-direction: column;
    gap: 10px;
  }
}
</style>
