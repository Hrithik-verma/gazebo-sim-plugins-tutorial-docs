document.addEventListener("click", (e) => {
  const a = e.target.closest(".md-sidebar--primary a");
  if (!a) return;

  const drawer = document.getElementById("__drawer");
  if (drawer) drawer.checked = false;
});
