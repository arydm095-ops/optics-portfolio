document.querySelectorAll('[data-copy-link]').forEach(button => {
  button.addEventListener('click', async () => {
    const url = location.href.split('#')[0];
    const toast = document.querySelector('.toast');
    try {
      await navigator.clipboard.writeText(url);
      toast.textContent = '链接已复制';
    } catch {
      toast.textContent = '请复制浏览器地址栏中的链接';
    }
    toast.classList.add('visible');
    clearTimeout(window.toastTimer);
    window.toastTimer = setTimeout(() => toast.classList.remove('visible'), 3200);
  });
});
