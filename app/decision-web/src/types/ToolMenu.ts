import { activeToolId } from '@/types/Manger';
import { resetAddEdgeState, initAddEdgeState } from '@/types/extensions/ToolMenu/AddEdge';
import { resetAddNodeState, initAddNodeState } from '@/types/extensions/ToolMenu/AddNode';
export { activeToolId };
import { exportRule, uploadRuleFile } from '@/types/extensions/MangerTool/Export';
// 选择工具方法
export const selectTool = (toolId: string) => {
  console.log('选择工具', toolId);
  activeToolId.value = toolId;
  // emit('tool-selected', toolId);

  // 工具对应的具体行为可以在父组件中定义
  // 这里根据工具ID触发相应的事件
  switch(toolId) {
    case 'cursor':
      console.log('切换到选择工具');
      resetAddEdgeState();
      resetAddNodeState();
      break;
    case 'renderTaskNode':
      console.log('添加任务节点');
      resetAddEdgeState();
      initAddNodeState();
      break;
    case 'renderNodeEdge':
      console.log('添加连接线');
      resetAddNodeState();
      initAddEdgeState();
      break;
    case 'renderNodeGroup':
      console.log('打开任务编组');
      resetAddEdgeState();
      resetAddNodeState();
      break;
    case 'map':
      console.log('打开航点地图');
      resetAddEdgeState();
      resetAddNodeState();
      break;
    case 'help':
      console.log('显示帮助信息');
      resetAddEdgeState();
      resetAddNodeState();
      break;
    case 'info':
      console.log('显示版本信息');
      resetAddEdgeState();
      resetAddNodeState();
      break;
    case 'save':
      console.log('保存数据');
      resetAddEdgeState();
      resetAddNodeState();
      exportRule();
      activeToolId.value = 'cursor';
      break;
    case 'sync':
      console.log('导入数据');
      resetAddEdgeState();
      resetAddNodeState();
      uploadRuleFile();
      activeToolId.value = 'cursor';
      break;
  }
};
