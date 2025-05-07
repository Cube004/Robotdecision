import { ref } from 'vue';
import ConfirmWindow from '@/components/ConfirmWindow.vue';

export const confirmVisible = ref(false);
export const confirmMessage = ref('您确定要执行此操作吗？');
export const confirmTitle = ref('操作确认');
export const confirmWindowRef = ref<InstanceType<typeof ConfirmWindow> | null>(null);

// 显示确认窗口的方法二：使用Promise API
export const showConfirmPromise = async (message: string, title: string = '确认提示'): Promise<boolean> => {
  confirmMessage.value = message;
  confirmTitle.value = title;
  if (!confirmWindowRef.value) {
    console.error('confirmWindowRef未设置');
    return false;
  }

  try {
    return await confirmWindowRef.value.showConfirm();
  } catch (error) {
    console.error('确认对话框发生错误', error);
    return false;
  }
};

// 处理确认结果
export const handleConfirmResult = (result: boolean) => {
  return result;
};