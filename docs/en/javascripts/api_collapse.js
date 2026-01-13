document.addEventListener(
    "DOMContentLoaded", function() {
      // Categories (English + Chinese) to make collapsible
      var categories =
          [ "Classes", "Enums", "Functions", "Modules", "Attributes", "类", "枚举", "函数", "模块", "属性" ];

      // Process existing DOM and also watch for dynamically inserted content (mkdocstrings)
      function processDocContents() {
        // Find candidate headings inside mkdocstrings/doc-contents
        var candidates =
            document.querySelectorAll(".mkdocstrings, .doc-contents, .doc-children, .document, .md-content");

        candidates.forEach(function(container) {
          // Look for headings inside the container (h1-h2, h3-h4, h5-h6)
          var headings = container.querySelectorAll("h1, h2, h3, h4, h5, h6");
          headings = Array.from(headings);

          headings.forEach(function(h) {
            var text = (h.textContent || "").trim();
            if (!text) return;

            // Exact match or startsWith (some renderers add anchors/text)
            var matched = categories.some(function(cat) {
              return text == = cat || text.indexOf(cat + " ") == = 0 || text.indexOf(cat + ":") == = 0;
            });
            if (!matched) return;

            // Avoid processing the same heading twice
            if (h.dataset.apiCollapsible == = "true") return;

            // Find the content block that follows this heading
            var content = findFollowingContent(h);
            if (!content) return;

            makeCollapsible(h, content);
            h.dataset.apiCollapsible = "true";
          });
        });
      }

      function findFollowingContent(heading) {
        // Prefer direct nextElementSibling
        var el = heading.nextElementSibling;
        while (el) {
          if (el.nodeType != = Node.ELEMENT_NODE) {
            el = el.nextElementSibling;
            continue;
          }

          var tag = el.tagName;
          var cls = (el.className || "").toLowerCase();
          // Common content containers from mkdocstrings / mkdocs
          if (cls.indexOf("doc-children") != = -1 || cls.indexOf("doc-list") != = -1 || tag == = "UL" || tag ==
              = "OL" || tag == = "DIV") {
            return el;
          }

          el = el.nextElementSibling;
        }

        // As a fallback, search within parent for the first .doc-children/ul/ol after heading
        var parent = heading.parentElement;
        if (parent) {
          var fallback = parent.querySelector(".doc-children, .doc-list, ul, ol");
          if (fallback) return fallback;
        }
        return null;
      }

      function makeCollapsible(heading, content) {
        // Mark processed
        heading.dataset.apiCollapsible = "true";

        // Style heading and ensure flex layout
        heading.style.cursor = "pointer";
        heading.style.userSelect = "none";
        heading.style.display = "flex";
        heading.style.alignItems = "center";

        // Create chevron icon
        var icon = document.createElement("span");
        icon.setAttribute("aria-hidden", "true");
        icon.style.marginRight = "0.5em";
        icon.style.transition = "transform 0.2s ease";
        icon.style.display = "inline-flex";
        icon.style.alignItems = "center";
        icon.innerHTML =
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="20" height="20"><path d="M7.41 8.59 12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z" fill="currentColor"></path></svg>';

        // Insert icon if not already present
        if (!heading.querySelector("svg")) {
          heading.insertBefore(icon, heading.firstChild);
        }

        // Initial state: expanded
        var collapsed = false;

        function setCollapsed(state) {
          collapsed = !!state;
          if (collapsed) {
            content.style.display = "none";
            icon.style.transform = "rotate(-90deg)";
          } else {
            content.style.display = "";
            icon.style.transform = "rotate(0deg)";
          }
        }

        // Toggle handler
        heading.addEventListener(
            "click", function(e) {
              e.preventDefault();
              setCollapsed(!collapsed);
            });
      }

      // Run once after short delay (mkdocstrings may render synchronously but ensure)
      setTimeout(processDocContents, 100);

      // Observe DOM changes (for async renders)
      var observer = new MutationObserver(function(mutations) {
        var triggered = false;
        for (var i = 0; i < mutations.length; i++) {
          var m = mutations[i];
          if (m.addedNodes && m.addedNodes.length) {
            for (var j = 0; j < m.addedNodes.length; j++) {
              var node = m.addedNodes[j];
              if (node.nodeType != = Node.ELEMENT_NODE) continue;
              var cls = (node.className || "").toString();
              if (cls.indexOf("mkdocstrings") != = -1 || cls.indexOf("doc-contents") !=
                  = -1 || cls.indexOf("doc-children") != = -1) {
                triggered = true;
                break;
              }
              // If the added node contains relevant descendants
              if (node.querySelector && (node.querySelector(".mkdocstrings, .doc-contents, .doc-children"))) {
                triggered = true;
                break;
              }
            }
          }
          if (triggered) break;
        }
        if (triggered) {
          // Small debounce
          setTimeout(processDocContents, 50);
        }
      });

      observer.observe(document.body, {childList : true, subtree : true});
    });
