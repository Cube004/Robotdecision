import { ref } from 'vue';
import { mapContainer } from './MapPage';
import { areas } from '@/types/Manger';
import { showAreaMenu, areaMenuStyle, selectedArea, areaName, areaColor } from './MapPage';

// ----------------------------区域菜单配置----------------------------
export const showAreaColorPicker = ref(false);
export const showAreaDeleteConfirm = ref(false);

// 全局点击事件处理，关闭区域菜单
export const handleOutsideClick = (event: MouseEvent) => {
  // 如果点击的是区域菜单或其子元素，不关闭菜单
  const areaMenu = document.getElementById('areaMenu');
  if (areaMenu && areaMenu.contains(event.target as Node)) {
    return;
  }

  // 如果点击的是区域对象，通过事件委托处理，不在这里关闭
  const target = event.target as HTMLElement;
  if (target.classList.contains('area') ||
      (target.parentElement && target.parentElement.classList.contains('area'))) {
    return;
  }

  // 其他情况关闭区域菜单
  showAreaMenu.value = false;
};

// 区域颜色选项
export const areaColorOptions = ref([
  'rgba(59, 130, 246, 0.2)', // 蓝色
  'rgba(239, 68, 68, 0.2)',  // 红色
  'rgba(34, 197, 94, 0.2)',  // 绿色
  'rgba(168, 85, 247, 0.2)', // 紫色
  'rgba(249, 115, 22, 0.2)', // 橙色
  'rgba(234, 179, 8, 0.2)',  // 黄色
  'rgba(107, 114, 128, 0.2)' // 灰色
]);


// 更新区域菜单位置
export function updateAreaMenuPosition(x: number, y: number) {
  if (!mapContainer.value) {
    return;
  }

  const containerRect = mapContainer.value.getBoundingClientRect();
  const containerWidth = containerRect.width;
  const containerHeight = containerRect.height;

  // 菜单尺寸
  const menuWidth = 320;
  // 菜单高度需要考虑颜色选择器和删除确认框展开时的高度
  const baseMenuHeight = 200;  // 基本菜单高度
  const colorPickerHeight = 150;  // 颜色选择器高度
  const deleteConfirmHeight = 150;  // 删除确认框高度

  // 根据当前菜单状态估算高度
  let estimatedHeight = baseMenuHeight;
  if (showAreaColorPicker.value) {
    estimatedHeight += colorPickerHeight;
  }
  if (showAreaDeleteConfirm.value) {
    estimatedHeight += deleteConfirmHeight;
  }

  // 计算初始位置，将触发点定位在菜单上方中心位置
  let menuX = x - menuWidth / 2;
  let menuY = y - 10;  // 菜单略微向上偏移以避免鼠标遮挡

  // 检查右边界
  if (menuX + menuWidth > containerWidth) {
    menuX = containerWidth - menuWidth - 10;
  }

  // 确保菜单不超出左侧边界
  if (menuX < 10) {
    menuX = 10;
  }

  // 检查底部边界 - 这是关键修复
  if (menuY + estimatedHeight > containerHeight) {
    menuY = containerHeight - estimatedHeight - 10;

    // 如果向上调整后菜单顶部超出容器，则固定在顶部
    if (menuY < 10) {
      menuY = 10;
    }
  }

  // 设置菜单样式
  areaMenuStyle.value = {
    top: `${menuY}px`,
    left: `${menuX}px`,
    display: 'block'
  };
}

// --------------------------------区域菜单功能--------------------------------
export const toggleAreaColorPicker = () => {
  showAreaColorPicker.value = !showAreaColorPicker.value;
  showAreaDeleteConfirm.value = false;
};

export const toggleAreaDeleteConfirm = () => {
  showAreaDeleteConfirm.value = !showAreaDeleteConfirm.value;
  showAreaColorPicker.value = false;
};

export const selectAreaColor = (color: string) => {
  if (selectedArea.value) {
    selectedArea.value.color = color;
    areaColor.value = color;
  }
};

export const updateAreaName = () => {
  if (selectedArea.value && areaName.value) {
    selectedArea.value.name = areaName.value;
  }
};

export const deleteArea = () => {
  if (selectedArea.value) {
    const id = selectedArea.value.id;
    const index = areas.value.findIndex(a => a.id === id);
    if (index !== -1) {
      areas.value.splice(index, 1);
      showAreaMenu.value = false;
    }
  }
};

// --------------------------------区域菜单功能--------------------------------
