<template>
  <div class="edit-menu" :class="{ 'open': panelOpen }">
    <div class="edit-menu-content" v-if="panelOpen">
      <!-- 顶部导航栏和标题 -->
      <div class="menu-header">
        <h3>任务编组</h3>
        <button class="close-button" @click="close" title="关闭">×</button>
      </div>

      <!-- 编组列表区域 -->
      <div class="edit-area">
        <div class="setting-card">
          <div class="card-header">
            <h4>编组列表</h4>
            <button class="add-button" @click="createNewGroup" title="创建新编组">
              <span>+</span>
            </button>
          </div>

          <div v-if="NodeGroups.length === 0" class="empty-hint">
            暂无编组，请点击右上角"+"按钮创建
          </div>
          <div v-else class="group-list">
            <div
              v-for="group in NodeGroups"
              :key="group.id"
              class="group-item"
              :class="{ 'selected': selectedGroup?.id === group.id }"
              @click="selectGroup(group)"
            >
              <div class="group-color" :style="{ backgroundColor: group.color }"></div>
              <div class="group-info">
                <div class="group-name">{{ group.name }}</div>
                <div class="group-meta">ID: {{ group.id }} | 节点数: {{ group.nodesId.length }}</div>
              </div>
              <button class="delete-btn" @click.stop="deleteGroup(group)">
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M18 6L6 18" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                  <path d="M6 6L18 18" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                </svg>
              </button>
            </div>
          </div>
        </div>

        <!-- 编辑部分 -->
        <div v-if="selectedGroup">
          <!-- 基本属性设置 -->
          <div class="setting-card">
            <h4>基本属性</h4>
            <div class="form-group">
              <label>名称:</label>
              <input type="text" v-model="editingName" placeholder="请输入编组名称" />
            </div>
            <div class="form-group">
              <label>颜色:</label>
              <div class="color-control">
                <input type="color" v-model="editingColor" />
                <span class="color-code">{{ editingColor }}</span>
              </div>
            </div>
            <div class="form-group">
              <label>透明度:</label>
              <div class="slider-control">
                <input type="range" v-model="editingOpacity" min="0" max="1" step="0.01" />
                <span class="slider-value">{{ (editingOpacity * 100).toFixed(0) }}%</span>
              </div>
            </div>
          </div>

          <!-- 扩展选项（循环、重置时间、反转） -->
          <div class="setting-card">
            <h4>扩展选项</h4>
            <div class="form-group">
              <label>循环:</label>
              <div class="toggle-switch">
                <input type="checkbox" id="loop-toggle" v-model="editingConfig.Loop" />
                <label for="loop-toggle" class="toggle-label">
                  <span class="toggle-inner"></span>
                  <span class="toggle-switch-label">{{ editingConfig.Loop ? '开启' : '关闭' }}</span>
                </label>
              </div>
            </div>
            <div class="form-group">
              <label>重置时间:</label>
              <div class="slider-control">
                <input
                  type="range"
                  v-model.number="editingConfig.ResetTime"
                  min="0"
                  max="30"
                  step="0.5"
                  :disabled="!editingConfig.Loop"
                />
                <span class="slider-value">{{ editingConfig.ResetTime.toFixed(1) }}秒</span>
              </div>
            </div>
            <div class="form-group">
              <label>反转执行顺序:</label>
              <div class="toggle-switch">
                <input type="checkbox" id="reverse-toggle" v-model="editingConfig.Reverse" :disabled="!editingConfig.Loop" />
                <label for="reverse-toggle" class="toggle-label">
                  <span class="toggle-inner"></span>
                  <span class="toggle-switch-label">{{ editingConfig.Reverse ? '开启' : '关闭' }}</span>
                </label>
              </div>
            </div>
            <button class="save-button" @click="saveGroup">保存更改</button>
          </div>

          <!-- 节点管理 -->
          <div class="setting-card">
            <h4>节点管理</h4>
            <div class="node-selection">
              <h5>已选节点 <span class="count-badge">{{ selectedGroup.nodesId.length }}</span></h5>
              <div class="node-chips">
                <div
                  v-for="nodeId in selectedGroup.nodesId"
                  :key="nodeId"
                  class="node-chip"
                >
                  节点 #{{ nodeId }}
                  <button class="remove-node" @click="removeNodeFromGroup(nodeId)">×</button>
                </div>
              </div>

              <h5>添加节点</h5>
              <div class="add-node-section">
                <select v-model="selectedNodeToAdd" class="node-select">
                  <option value="">-- 选择节点 --</option>
                  <option
                    v-for="node in availableNodes"
                    :key="node.id.value"
                    :value="node.id.value"
                  >
                    {{ node.text.content }} (#{{ node.id.value }})
                  </option>
                </select>
                <button
                  class="add-node-btn"
                  :disabled="!selectedNodeToAdd"
                  @click="addNodeToGroup"
                >
                  添加节点
                </button>
              </div>
            </div>
          </div>
        </div>

        <!-- 提示选择编组 -->
        <div v-if="!selectedGroup && NodeGroups.length > 0" class="empty-selection">
          <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" stroke="#a0aec0" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
          <p>请从列表中选择一个编组进行编辑</p>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, watch } from 'vue';
import { NodeGroups, nodes } from '@/types/Manger';
import type { NodeGroup, NodeGroupConfig } from '@/types/Node';

// 接收外部传入的isOpen属性
const props = defineProps<{
  isOpen: boolean
}>();

// 面板是否打开 - 内部状态
const panelOpen = ref(false);
const emit = defineEmits(['close', 'node-group-click']);

// 监听props的isOpen变化
watch(() => props.isOpen, (newValue) => {
  panelOpen.value = newValue;
  if (panelOpen.value) {
    emit('node-group-click');
  }
}, { immediate: true });

// 当前选中的组
const selectedGroup = ref<NodeGroup | null>(null);

// 编辑中的字段
const editingName = ref('');
const editingColor = ref('#3B82F6'); // 不含透明度的颜色
const editingOpacity = ref(0.5); // 透明度，0-1之间的值
const editingConfig = ref<NodeGroupConfig>({
  Loop: false,
  ResetTime: 0,
  Reverse: false
});

// 将颜色和透明度合并成rgba格式
const getFullColor = (color: string, opacity: number): string => {
  // 将hex颜色转换为rgb
  const hex = color.replace('#', '');
  const r = parseInt(hex.substring(0, 2), 16);
  const g = parseInt(hex.substring(2, 4), 16);
  const b = parseInt(hex.substring(4, 6), 16);

  // 返回rgba格式
  return `rgba(${r}, ${g}, ${b}, ${opacity})`;
};

// 将rgba或hex+alpha格式的颜色拆分为纯色和透明度
const splitColor = (fullColor: string): { color: string, opacity: number } => {
  // 检查是否为rgba格式
  if (fullColor.startsWith('rgba')) {
    const parts = fullColor.match(/rgba\((\d+),\s*(\d+),\s*(\d+),\s*([\d.]+)\)/);
    if (parts) {
      const [, r, g, b, a] = parts;
      const hex = `#${Number(r).toString(16).padStart(2, '0')}${Number(g).toString(16).padStart(2, '0')}${Number(b).toString(16).padStart(2, '0')}`;
      return { color: hex, opacity: parseFloat(a) };
    }
  }

  // 检查是否为带透明度的hex格式（如#FF000080）
  if (fullColor.length === 9 && fullColor.startsWith('#')) {
    const color = fullColor.substring(0, 7);
    const alphaHex = fullColor.substring(7);
    const opacity = parseInt(alphaHex, 16) / 255;
    return { color, opacity };
  }

  // 返回默认值
  return { color: fullColor, opacity: 1.0 };
};

// 选择要添加的节点
const selectedNodeToAdd = ref<number | ''>('');

// 关闭面板
const close = () => {
  panelOpen.value = false;
  selectedGroup.value = null;
  emit('close');
};

// 选择组进行编辑
const selectGroup = (group: NodeGroup) => {
  selectedGroup.value = group;
  editingName.value = group.name;

  // 拆分颜色和透明度
  const { color, opacity } = splitColor(group.color);
  editingColor.value = color;
  editingOpacity.value = opacity;

  editingConfig.value = { ...group.config };
};

// 保存组的修改
const saveGroup = () => {
  if (selectedGroup.value) {
    // 合并颜色和透明度
    const fullColor = getFullColor(editingColor.value, editingOpacity.value);

    // 更新组信息
    const groupIndex = NodeGroups.value.findIndex(g => g.id === selectedGroup.value!.id);
    if (groupIndex >= 0) {
      NodeGroups.value[groupIndex] = {
        ...NodeGroups.value[groupIndex],
        name: editingName.value,
        color: fullColor,
        config: { ...editingConfig.value }
      };
    }
  }
};

// 创建新组
const createNewGroup = () => {
  // 生成新ID - 找到当前最大ID并加1
  const newId = NodeGroups.value.length > 0
    ? Math.max(...NodeGroups.value.map(g => g.id)) + 1
    : 0;

  // 创建新组
  const newGroup: NodeGroup = {
    id: newId,
    name: `编组 ${newId}`,
    color: getRandomColor(),
    nodesId: [],
    config: {
      Loop: false,
      ResetTime: 0,
      Reverse: false
    }
  };

  // 添加到组列表
  NodeGroups.value.push(newGroup);

  // 选择新创建的组
  selectGroup(newGroup);
};

// 删除组
const deleteGroup = (group: NodeGroup) => {
  if (confirm(`确定要删除编组 "${group.name}" 吗？`)) {
    const index = NodeGroups.value.findIndex(g => g.id === group.id);
    if (index >= 0) {
      NodeGroups.value.splice(index, 1);

      // 如果删除的是当前选中的组，清除选择
      if (selectedGroup.value?.id === group.id) {
        selectedGroup.value = null;
      }
    }
  }
};

// 可用于添加的节点
const availableNodes = computed(() => {
  if (!selectedGroup.value) return [];
  // 返回未包含在当前组内的所有节点
  return nodes.value.filter(node => !selectedGroup.value!.nodesId.includes(node.id.value));
});

// 添加节点到组
const addNodeToGroup = () => {
  if (selectedGroup.value && selectedNodeToAdd.value !== '') {
    const nodeId = Number(selectedNodeToAdd.value);
    // 确保节点存在且不在当前组内
    if (nodes.value.some(node => node.id.value === nodeId) &&
        !selectedGroup.value.nodesId.includes(nodeId)) {
      // 添加节点ID到组
      selectedGroup.value.nodesId.push(nodeId);
      // 重置选择
      selectedNodeToAdd.value = '';
    }
  }
};

// 从组中移除节点
const removeNodeFromGroup = (nodeId: number) => {
  if (selectedGroup.value) {
    const index = selectedGroup.value.nodesId.indexOf(nodeId);
    if (index >= 0) {
      selectedGroup.value.nodesId.splice(index, 1);
    }
  }
};

// 生成随机颜色
const getRandomColor = () => {
  const colors = [
    'rgba(59, 130, 246, 0.4)', // 蓝色
    'rgba(16, 185, 129, 0.4)', // 绿色
    'rgba(245, 158, 11, 0.4)', // 橙色
    'rgba(239, 68, 68, 0.4)', // 红色
    'rgba(139, 92, 246, 0.4)', // 紫色
    'rgba(236, 72, 153, 0.4)', // 粉色
    'rgba(99, 102, 241, 0.4)', // 靛蓝色
    'rgba(217, 119, 6, 0.4)'  // 琥珀色
  ];
  return colors[Math.floor(Math.random() * colors.length)];
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
  margin-bottom: 20px;
}

.setting-card:hover {
  box-shadow: 0 5px 15px rgba(0, 0, 0, 0.07);
}

.setting-card h4 {
  margin: 0 0 16px 0;
  font-size: 16px;
  font-weight: 600;
  color: #3B82F6;
}

.setting-card h5 {
  margin: 16px 0 8px 0;
  font-size: 14px;
  font-weight: 500;
  color: #4b5563;
  display: flex;
  align-items: center;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
}

.add-button {
  width: 28px;
  height: 28px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #e7f1ff;
  color: #3B82F6;
  border: none;
  border-radius: 6px;
  font-size: 18px;
  font-weight: bold;
  cursor: pointer;
  transition: all 0.2s ease;
}

.add-button:hover {
  background-color: #d3e5ff;
  transform: translateY(-1px);
}

/* 空列表提示 */
.empty-hint {
  padding: 16px;
  text-align: center;
  color: #6c757d;
  background-color: #f9f9f9;
  border-radius: 8px;
  border: 1px dashed #dee2e6;
}

.empty-selection {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 40px 20px;
  text-align: center;
  color: #6c757d;
}

.empty-selection svg {
  margin-bottom: 16px;
  opacity: 0.7;
}

.empty-selection p {
  font-size: 15px;
  margin: 0;
}

/* 组列表 */
.group-list {
  max-height: 240px;
  overflow-y: auto;
  border: 1px solid #eee;
  border-radius: 8px;
}

.group-item {
  display: flex;
  align-items: center;
  padding: 12px 16px;
  border-bottom: 1px solid #eee;
  cursor: pointer;
  transition: background-color 0.2s;
}

.group-item:last-child {
  border-bottom: none;
}

.group-item:hover {
  background-color: #f5f5f5;
}

.group-item.selected {
  background-color: #ebf5ff;
}

.group-color {
  width: 16px;
  height: 16px;
  border-radius: 4px;
  margin-right: 12px;
}

.group-info {
  flex: 1;
}

.group-name {
  font-weight: 500;
}

.group-meta {
  font-size: 12px;
  color: #6c757d;
  margin-top: 4px;
}

.delete-btn {
  width: 28px;
  height: 28px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: transparent;
  color: #f87171;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.2s;
}

.delete-btn:hover {
  background-color: #fee2e2;
}

/* 表单元素 */
.form-group {
  margin-bottom: 16px;
  display: flex;
  align-items: center;
}

.form-group label {
  width: 120px;
  flex-shrink: 0;
  font-weight: 500;
  color: #495057;
  display: flex;
  height: 40px;
  align-items: center;
}

.form-group input[type="text"],
.form-group input[type="number"] {
  flex: 1;
  padding: 0 14px;
  background-color: #f8f9fa;
  border: 1px solid #dee2e6;
  border-radius: 8px;
  font-size: 14px;
  color: #212529;
  transition: all 0.2s ease;
  height: 40px;
}

.form-group input[type="text"]:focus,
.form-group input[type="number"]:focus {
  outline: none;
  background-color: white;
  border-color: #80bdff;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.15);
}

/* 颜色选择器 */
.color-control {
  display: flex;
  align-items: center;
  gap: 15px;
  flex: 1;
  height: 40px;
}

input[type="color"] {
  width: 40px;
  height: 40px;
  border: 1px solid #dee2e6;
  border-radius: 8px;
  cursor: pointer;
  background: none;
  padding: 3px;
  overflow: hidden;
  flex-shrink: 0;
}

input[type="color"]::-webkit-color-swatch {
  border: none;
  border-radius: 6px;
}

.color-code {
  flex: 1;
  font-family: monospace;
  font-size: 14px;
  color: #495057;
  background-color: #f1f3f5;
  padding: 8px 12px;
  border-radius: 6px;
  text-transform: uppercase;
  height: 36px;
  display: flex;
  align-items: center;
}

/* 滑块控制 */
.slider-control {
  display: flex;
  align-items: center;
  gap: 15px;
  flex: 1;
  height: 40px;
}

input[type="range"] {
  flex: 1;
  height: 6px;
  appearance: none;
  background-color: #e9ecef;
  border-radius: 3px;
  cursor: pointer;
}

input[type="range"]::-webkit-slider-thumb {
  appearance: none;
  width: 18px;
  height: 18px;
  background-color: #3B82F6;
  border-radius: 50%;
  cursor: pointer;
  border: 2px solid white;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

input[type="range"]:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.slider-value {
  min-width: 60px;
  text-align: center;
  font-family: monospace;
  font-size: 14px;
  color: #495057;
  background-color: #f1f3f5;
  padding: 6px 10px;
  border-radius: 6px;
  /* height: 36px; */
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

/* 切换开关 */
.toggle-switch {
  position: relative;
  display: flex;
  flex: 1;
  align-items: center;
  height: 40px;
}

.toggle-switch input {
  opacity: 0;
  width: 0;
  height: 0;
  position: absolute;
}

.toggle-label {
  display: flex;
  align-items: center;
  cursor: pointer;
  height: 24px;
}

.toggle-inner {
  position: relative;
  display: inline-block;
  width: 48px;
  height: 24px;
  background-color: #e9ecef;
  border-radius: 12px;
  margin-right: 12px;
  transition: all 0.3s;
  flex-shrink: 0;
}

.toggle-inner:before {
  content: '';
  position: absolute;
  width: 18px;
  height: 18px;
  left: 3px;
  bottom: 3px;
  background-color: white;
  border-radius: 50%;
  transition: 0.3s;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

input:checked + .toggle-label .toggle-inner {
  background-color: #3B82F6;
}

input:checked + .toggle-label .toggle-inner:before {
  transform: translateX(24px);
}

input:disabled + .toggle-label {
  opacity: 0.5;
  cursor: not-allowed;
}

.toggle-switch-label {
  font-size: 14px;
  color: #495057;
  line-height: 24px;
  display: inline-block;
  vertical-align: middle;
}

/* 数量徽章 */
.count-badge {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  min-width: 20px;
  height: 20px;
  padding: 0 6px;
  background-color: #e7f1ff;
  color: #3B82F6;
  border-radius: 10px;
  font-size: 12px;
  font-weight: 600;
  margin-left: 8px;
}

/* 节点管理 */
.node-selection {
  margin-top: 8px;
}

.node-chips {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  max-height: 120px;
  overflow-y: auto;
  padding: 10px;
  background-color: #f8f9fa;
  border: 1px solid #eee;
  border-radius: 8px;
}

.node-chip {
  display: flex;
  align-items: center;
  background-color: #e7f1ff;
  border-radius: 16px;
  padding: 6px 12px;
  font-size: 12px;
  color: #3B82F6;
}

.remove-node {
  background: none;
  border: none;
  margin-left: 6px;
  font-size: 14px;
  font-weight: bold;
  cursor: pointer;
  color: #4b5563;
}

.remove-node:hover {
  color: #ef4444;
}

.node-select {
  width: 100%;
  padding: 12px 14px;
  background-color: #f8f9fa;
  border: 1px solid #dee2e6;
  border-radius: 8px;
  font-size: 14px;
  color: #212529;
  margin-bottom: 10px;
  appearance: none;
  background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='16' height='16' viewBox='0 0 24 24' fill='none' stroke='%236B7280' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3E%3Cpolyline points='6 9 12 15 18 9'%3E%3C/polyline%3E%3C/svg%3E");
  background-repeat: no-repeat;
  background-position: right 12px center;
  padding-right: 36px;
}

.node-select:focus {
  outline: none;
  border-color: #80bdff;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.15);
}

.add-node-btn {
  width: 100%;
  padding: 12px;
  background-color: #3B82F6;
  color: white;
  border: none;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
}

.add-node-btn:hover:not(:disabled) {
  background-color: #2563EB;
  transform: translateY(-1px);
}

.add-node-btn:disabled {
  background-color: #d1d5db;
  cursor: not-allowed;
}

/* 保存按钮 */
.save-button {
  background-color: #3B82F6;
  color: white;
  border: none;
  padding: 12px;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  width: 100%;
  margin-top: 16px;
}

.save-button:hover {
  background-color: #2563EB;
  transform: translateY(-1px);
}

/* 移动设备适配 */
@media (max-width: 640px) {
  .edit-menu {
    width: 100%;
    right: -100%;
  }

  .form-group {
    flex-direction: column;
    align-items: flex-start;
  }

  .form-group label {
    width: 100%;
    margin-bottom: 8px;
  }
}
</style>
