import { ref } from 'vue';
import { activeToolId } from '@/types/ToolMenu';
import { createNewPoint, selectPointNearby, resetMenuState } from './PointMenu';
import { Point } from '@/types/Point';
import { getScale, calculateWaypoint } from './PointMenu';
import { MapSettingsPoints, showError, areas } from '@/types/Manger';
import { Area } from '@/types/Area';

export const mapContainer = ref<HTMLElement | null>(null);
export const showMap = ref(false);
export const showPointMenu = ref(false);
export const pointMenuStyle = ref({
  top: '0px',
  left: '0px',
  display: 'none'
});
export const mousePosition = ref({ x: 0, y: 0 });

// ----------------------------地图初始化设置----------------------------

// 为悬浮菜单添加响应式变量
export const showFloatingMenu = ref(false); // 默认显示悬浮菜单内容
export const showMapTools = ref(true); // 显示地图工具菜单
export const isDragging = ref(false); // 是否正在拖动
export const menuPosition = ref({ top: 20, left: 20 }); // 菜单位置
export const activeTab = ref(0); // 当前激活的标签页，默认显示第一个标签页

export const previewPoints = ref<Point | null>(null);
export const isPlacingSettingsPoint = ref(false);



// 区域菜单相关变量
export const isCreatingArea = ref(false);
export const areaFirstPoint = ref<{x: number, y: number} | null>(null);
export const showAreaMenu = ref(false);
export const areaMenuStyle = ref({
  top: '0px',
  left: '0px',
  display: 'none'
});
export const selectedArea = ref<Area | null>(null);
export const areaName = ref('');
export const areaColor = ref('rgba(59, 130, 246, 0.2)');
// 创建区域对象
export const createArea = () => {
  isCreatingArea.value = true;
  areaFirstPoint.value = null;
  showFloatingMenu.value = false;
};


export const setStartingArea = () => {
  isPlacingSettingsPoint.value = true;
  showFloatingMenu.value = false;
  previewPoints.value = new Point(
    0,
    { x: 0, y: 0 },
    { x: -1, y: -1 },
    '#000000',
    '启动点',
    14,
    '#000000',
    { x: 1, y: 1 }
  );
};
export const setTopLeftCorner = () => {
  isPlacingSettingsPoint.value = true;
  showFloatingMenu.value = false;
  previewPoints.value = new Point(
    1,
    { x: 0, y: 0 },
    { x: -1, y: -1 },
    '#000000',
    'LeftTop',
    14,
    '#000000',
    { x: 1, y: 1 }
  );
};
export const setBottomRightCorner = () => {
  isPlacingSettingsPoint.value = true;
  showFloatingMenu.value = false;
  previewPoints.value = new Point(
    2,
    { x: 0, y: 0 },
    { x: -1, y: -1 },
    '#000000',
    'RightBottom',
    14,
    '#000000',
    { x: 1, y: 1 }
  );
};

export const applyMapSize = () => {
  calculateWaypoint();
};

// ----------------------------地图点击事件----------------------------

// 地图点击事件
export function handleMapClick(event: MouseEvent) {
  if (!(event.target instanceof HTMLImageElement)) return;

  const rect = event.target.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;

  // 关闭区域菜单
  showAreaMenu.value = false;

  // 处理区域创建
  if (isCreatingArea.value) {
    if (!areaFirstPoint.value) {
      // 第一次点击，记录左上角点
      areaFirstPoint.value = { x, y };
      return;
    } else {
      // 第二次点击，创建区域
      const { x: x1, y: y1 } = areaFirstPoint.value;
      // 确保坐标正确（左上和右下）
      const leftTopX = Math.min(x1, x);
      const leftTopY = Math.min(y1, y);
      const rightBottomX = Math.max(x1, x);
      const rightBottomY = Math.max(y1, y);

      // 创建一个新的区域并添加到数组
      const scale = getScale();
      if (!scale) return;
      const { scaleX, scaleY } = scale;
      const newArea = new Area(
        areas.value.length,
        '新区域',
        { x: leftTopX, y: leftTopY },
        { x: rightBottomX, y: rightBottomY },
        'rgba(59, 130, 246, 0.2)',
        { x: scaleX, y: scaleY }
      );
      areas.value.push(newArea);

      // 重置创建状态
      isCreatingArea.value = false;
      areaFirstPoint.value = null;
      return;
    }
  }

  // 放置预览点
  if (isPlacingSettingsPoint.value && previewPoints.value) {
    isPlacingSettingsPoint.value = false;
    previewPoints.value.position = { x, y };

    MapSettingsPoints.value = MapSettingsPoints.value.filter(
      points => points.id.value !== previewPoints.value?.id.value
    );
    const scale = getScale();
    if (!scale) return;
    const { scaleX, scaleY } = scale;
    previewPoints.value.scale = { x: scaleX, y: scaleY };
    MapSettingsPoints.value.push(previewPoints.value);

    previewPoints.value = null;
    return;
  }

  if (event.button === 2 || event.ctrlKey) { // 右键点击或Ctrl+点击
    // 创建新航点
    if (MapSettingsPoints.value.length === 3) {
      createNewPoint(x, y);
    }else{
      showError('请先使用航点计算工具初始化地图参数');
    }
  } else { // 左键点击
    // 选择航点（找到最近的点）或显示菜单
    selectPointNearby(x, y);
  }
};

// 关闭地图
export function closeMap() {
  showMap.value = false;
  resetMenuState();

  // 调用TypeScript中定义的关闭函数
  setTimeout(() => {
    activeToolId.value = 'cursor';
  }, 100);

};

// 更新点菜单位置
export function updatePointMenuPosition(x: number, y: number) {
  if (!mapContainer.value) {
    return;
  }

  const containerRect = mapContainer.value.getBoundingClientRect();
  const containerWidth = containerRect.width;
  const containerHeight = containerRect.height;
  console.log("containerWidth", containerWidth);
  console.log("containerHeight", containerHeight);

  // 菜单尺寸（基于设置的固定宽度）
  const menuWidth = 280; // 与CSS中设置的宽度保持一致
  const menuHeight = 320; // 估计高度，包括可能展开的子菜单

  // 计算初始位置
  let menuX = x;
  let menuY = y;

  // 检查右边界
  if (menuX + menuWidth > containerWidth) {
    menuX = x - menuWidth;
  }

  // 检查底部边界
  if (menuY + menuHeight > containerHeight) {
    menuY = y - menuHeight;
  }

  // 如果上面的调整使菜单超出左侧或顶部边界，则重新调整
  menuX = Math.max(10, menuX);
  menuY = Math.max(10, menuY);

  // 设置菜单样式
  pointMenuStyle.value = {
    top: `${menuY}px`,
    left: `${menuX}px`,
    display: 'block'
  };
};

