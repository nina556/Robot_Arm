import {
  ArrowDownToLine,
  ArrowUp,
  ArrowUpFromLine,
  Bot,
  Box,
  BrainCircuit,
  Camera,
  CircleCheck,
  CircleX,
  Cloud,
  CloudDownload,
  CornerDownLeft,
  createIcons,
  Crosshair,
  Focus,
  FoldHorizontal,
  Gauge,
  Grab,
  Hand,
  House,
  ListChecks,
  ListTree,
  MapPinPlus,
  Move3d,
  OctagonAlert,
  PackageOpen,
  PanelRight,
  PersonStanding,
  Play,
  Plus,
  Power,
  PowerOff,
  RefreshCw,
  RotateCcw,
  Route,
  Save,
  Scan,
  Search,
  ScanEye,
  ScrollText,
  ShieldAlert,
  ShieldCheck,
  SlidersHorizontal,
  Trash2,
  TriangleAlert,
  UnfoldHorizontal,
  View,
  WandSparkles
} from 'lucide';

const ICONS = {
  ArrowDownToLine,
  ArrowUp,
  ArrowUpFromLine,
  Bot,
  Box,
  BrainCircuit,
  Camera,
  CircleCheck,
  CircleX,
  Cloud,
  CloudDownload,
  CornerDownLeft,
  Crosshair,
  Focus,
  FoldHorizontal,
  Gauge,
  Grab,
  Hand,
  House,
  ListChecks,
  ListTree,
  MapPinPlus,
  Move3d,
  OctagonAlert,
  PackageOpen,
  PanelRight,
  PersonStanding,
  Play,
  Plus,
  Power,
  PowerOff,
  RefreshCw,
  RotateCcw,
  Route,
  Save,
  Scan,
  Search,
  ScanEye,
  ScrollText,
  ShieldAlert,
  ShieldCheck,
  SlidersHorizontal,
  Trash2,
  TriangleAlert,
  UnfoldHorizontal,
  View,
  WandSparkles
};

export function byId(id) {
  return document.getElementById(id);
}

export function refreshIcons() {
  createIcons({ icons: ICONS, attrs: { 'stroke-width': 1.8 } });
}

export function formatNumber(value, digits = 3) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : '--';
}

export function escapeHtml(value) {
  return String(value ?? '')
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#039;');
}

export function setStatus(id, label, value, state = 'neutral') {
  const node = byId(id);
  if (!node) return;
  const labelNode = node.querySelector('b');
  const valueNode = node.querySelector('span:last-child');
  if (labelNode) labelNode.textContent = label;
  if (valueNode) valueNode.textContent = value;
  node.dataset.state = state;
}

export function setBadge(id, text, state = 'neutral') {
  const node = byId(id);
  if (!node) return;
  node.textContent = text;
  node.dataset.state = state;
}

export function showToast(message, kind = 'good', timeout = 3600) {
  const stack = byId('toastStack');
  const item = document.createElement('div');
  const icon = kind === 'bad' ? 'circle-x' : kind === 'warn' ? 'triangle-alert' : 'circle-check';
  item.className = 'toast';
  item.dataset.kind = kind;
  item.innerHTML = `<i data-lucide="${icon}"></i><p>${escapeHtml(message)}</p>`;
  stack.appendChild(item);
  refreshIcons(item);
  window.setTimeout(() => item.remove(), timeout);
}

export function describeApiError(error) {
  const failure = error?.payload?.failure;
  if (!failure) return error?.message || '未知错误';
  const parts = [];
  if (failure.stage_label) parts.push(failure.stage_label);
  if (failure.title) parts.push(failure.title);
  const reason = failure.message || error?.message;
  if (reason && !parts.includes(reason)) parts.push(reason);
  if (failure.moveit_error_code !== undefined) {
    parts.push(`MoveIt=${failure.moveit_error_code}${failure.moveit_error_name ? ` ${failure.moveit_error_name}` : ''}`);
  }
  if (failure.hint) parts.push(failure.hint);
  return parts.filter(Boolean).join(' | ');
}

export function confirmAction(message, title = '确认操作', acceptText = '确认') {
  const dialog = byId('confirmDialog');
  byId('confirmTitle').textContent = title;
  byId('confirmMessage').textContent = message;
  byId('confirmAccept').textContent = acceptText;
  dialog.hidden = false;

  return new Promise(resolve => {
    const finish = accepted => {
      dialog.hidden = true;
      byId('confirmCancel').removeEventListener('click', cancel);
      byId('confirmAccept').removeEventListener('click', accept);
      dialog.removeEventListener('click', backdrop);
      window.removeEventListener('keydown', keyboard);
      resolve(accepted);
    };
    const cancel = () => finish(false);
    const accept = () => finish(true);
    const backdrop = event => {
      if (event.target === dialog) cancel();
    };
    const keyboard = event => {
      if (event.key === 'Escape') cancel();
      if (event.key === 'Enter') accept();
    };
    byId('confirmCancel').addEventListener('click', cancel);
    byId('confirmAccept').addEventListener('click', accept);
    dialog.addEventListener('click', backdrop);
    window.addEventListener('keydown', keyboard);
    byId('confirmCancel').focus();
  });
}

export async function runCommand(label, command, options = {}) {
  const { confirm, confirmTitle, success = `${label}完成`, quiet = false, onError = null } = options;
  if (document.body.dataset.busy) {
    showToast('已有命令正在执行，请等待完成', 'warn', 2200);
    return null;
  }
  if (confirm && !(await confirmAction(confirm, confirmTitle))) {
    return null;
  }

  byId('footerMessage').textContent = `${label}...`;
  document.body.dataset.busy = 'true';
  document.querySelectorAll('button').forEach(button => {
    if (!button.disabled) {
      button.dataset.busyDisabled = 'true';
      button.disabled = true;
    }
  });
  try {
    const result = await command();
    byId('footerMessage').textContent = success;
    if (!quiet) showToast(success);
    return result;
  } catch (error) {
    const message = `${label}失败：${describeApiError(error)}`;
    byId('footerMessage').textContent = message;
    showToast(message, 'bad', 9000);
    onError?.(error, message);
    return null;
  } finally {
    document.querySelectorAll('button[data-busy-disabled="true"]').forEach(button => {
      button.disabled = false;
      delete button.dataset.busyDisabled;
    });
    delete document.body.dataset.busy;
  }
}
