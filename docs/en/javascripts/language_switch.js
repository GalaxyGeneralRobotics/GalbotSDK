document.addEventListener(
    "DOMContentLoaded", function() {
      var header = document.querySelector(".md-header__inner");
      if (!header) return;

      var langBtn = document.createElement("a");
      langBtn.className = "md-header__button";
      langBtn.style.fontSize = "0.9rem";
      langBtn.style.width = "auto";
      langBtn.style.padding = "0 12px";
      langBtn.style.color = "inherit";
      langBtn.style.fontWeight = "bold";
      langBtn.style.textDecoration = "none";
      langBtn.style.display = "flex";
      langBtn.style.alignItems = "center";
      langBtn.style.justifyContent = "center";
      langBtn.style.cursor = "pointer";
      langBtn.style.border = "1px solid rgba(255,255,255,0.3)";
      langBtn.style.borderRadius = "4px";
      langBtn.style.height = "1.5rem";
      langBtn.style.marginTop = "0.6rem";  // Align vertically
      langBtn.style.marginRight = "10px";

      var path = window.location.pathname;
      var isEn = path.indexOf("/en/") !== -1;
      var isZh = path.indexOf("/zh/") !== -1;

      if (isEn) {
        langBtn.textContent = "中文";
        langBtn.href = path.replace("/en/", "/zh/");
        langBtn.title = "Switch to Chinese";
      } else if (isZh) {
        langBtn.textContent = "English";
        langBtn.href = path.replace("/zh/", "/en/");
        langBtn.title = "切换到英文";
      } else {
        return;
      }

      // Insert before the palette toggle
      var palette = document.querySelector("[data-md-component='palette']");
      if (palette) {
        var paletteContainer = palette.closest(".md-header__option");
        if (paletteContainer) {
          paletteContainer.parentNode.insertBefore(langBtn, paletteContainer);
        } else {
          palette.parentNode.insertBefore(langBtn, palette);
        }
      } else {
        // Fallback: Insert before the repository link
        var repo = document.querySelector(".md-header__source");
        if (repo) {
          repo.parentNode.insertBefore(langBtn, repo);
        } else {
          header.appendChild(langBtn);
        }
      }
    });