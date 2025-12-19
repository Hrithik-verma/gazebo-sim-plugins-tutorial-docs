document$.subscribe(() => {
  if (!window.mermaid) return;

  // reset processed flags so it reruns on instant navigation
  document.querySelectorAll('.mermaid[data-processed="true"]').forEach((el) => {
    el.removeAttribute('data-processed');
  });

  mermaid.initialize({ startOnLoad: false });

  if (typeof mermaid.run === 'function') {
    mermaid.run({ querySelector: '.mermaid' });
  } else if (typeof mermaid.init === 'function') {
    mermaid.init(undefined, document.querySelectorAll('.mermaid'));
  }
});
