import NodeBase, { NodeType } from './NodeBase';
import { extendNodeWithDrag } from './extensions/node/nodeDrag';

// 扩展 Node 类
extendNodeWithDrag();

// 为了更好的类型支持，我们结合静态和实例类型
interface NodeStatic {
  draggingNodeId: { value: number | null };
  startDrag(event: MouseEvent, node: NodeBase, nodeList: NodeBase[]): void;
}

// 扩展Node的构造函数类型，包含静态成员
type NodeConstructor = typeof NodeBase & NodeStatic;

// 导出扩展后的Node
const Node = NodeBase as NodeConstructor;
export { Node , NodeType};

// 再导出Node类型
export type Node = NodeBase;

// ----------------------------任务编组----------------------------
export interface NodeGroupConfig {
  Loop: boolean; // 是否循环, 当编组内所有节点都执行完成后, 是否重置完成状态
  ResetTime: number; // 重置时间, 用于重置编组的状态
  Reverse: boolean; // 是否反转, 当编组内所有节点都执行完成后, 是否反转执行顺序
}

export interface NodeGroup {
  id: number;
  name: string;
  color: string;
  nodesId: number[];
  config: NodeGroupConfig;
}
// ----------------------------任务编组----------------------------