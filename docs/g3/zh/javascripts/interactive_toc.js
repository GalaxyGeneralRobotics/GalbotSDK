/**
 * Interactive Dual ToC for MkDocs Material
 *
 * Creates an interactive three-column layout where:
 * - Left sidebar: Site navigation + H2 headings (type names) only
 * - Right sidebar: Shows member entries for the selected H2 type
 *
 * Requires toc.integrate to be enabled in mkdocs.yml
 */

(function() {
    'use strict';

    document.addEventListener('DOMContentLoaded', function() {
        const integratedToc = document.querySelector('.md-sidebar--primary .md-nav--secondary');
        if (!integratedToc) {
            return;
        }

        const rightSidebar = document.createElement('div');
        rightSidebar.className = 'md-sidebar md-sidebar--secondary';
        rightSidebar.setAttribute('data-md-component', 'toc');

        const scrollwrap = document.createElement('div');
        scrollwrap.className = 'md-sidebar__scrollwrap';

        const inner = document.createElement('div');
        inner.className = 'md-sidebar__inner';

        const detailToc = document.createElement('nav');
        detailToc.className = 'md-nav md-nav--secondary';
        detailToc.setAttribute('aria-label', 'Member functions');

        const detailTitle = document.createElement('label');
        detailTitle.className = 'md-nav__title';
        detailTitle.innerHTML = '<span class="md-nav__icon md-icon"></span> Member functions';
        detailToc.appendChild(detailTitle);

        const detailList = document.createElement('ul');
        detailList.className = 'md-nav__list';
        detailToc.appendChild(detailList);

        inner.appendChild(detailToc);
        scrollwrap.appendChild(inner);
        rightSidebar.appendChild(scrollwrap);

        const mainInner = document.querySelector('.md-main__inner');
        if (mainInner) {
            mainInner.appendChild(rightSidebar);
        }

        const h2Items = Array.from(integratedToc.querySelectorAll('.md-nav__list > .md-nav__item'));

        function isTypesSectionItem(h2Item) {
            const h2Link = h2Item.querySelector(':scope > .md-nav__link');
            const title = (h2Link ? h2Link.textContent : '').trim().toLowerCase();
            const href = (h2Link ? h2Link.getAttribute('href') : '') || '';
            return (
                title === 'types & enums' ||
                title === '类型与枚举' ||
                href.indexOf('#module-types-enums') >= 0
            );
        }

        function normalizeHash(href) {
            if (!href) return '';
            const idx = href.indexOf('#');
            if (idx < 0) return '';
            return decodeURIComponent(href.slice(idx));
        }

        function findTypeItemForHash(h2Item, hash) {
            const h3Nav = h2Item.querySelector(':scope > .md-nav');
            if (!h3Nav) return null;

            const h3Items = Array.from(h3Nav.querySelectorAll(':scope > .md-nav__list > .md-nav__item'));
            if (h3Items.length === 0) return null;
            if (!hash) return h3Items[0];

            for (const h3Item of h3Items) {
                const h3Link = h3Item.querySelector(':scope > .md-nav__link');
                const h3Hash = normalizeHash(h3Link ? h3Link.getAttribute('href') : '');
                if (h3Hash && h3Hash === hash) {
                    return h3Item;
                }

                const descendants = h3Item.querySelectorAll('a.md-nav__link');
                for (const link of descendants) {
                    const linkHash = normalizeHash(link.getAttribute('href'));
                    if (linkHash && linkHash === hash) {
                        return h3Item;
                    }
                }
            }

            return h3Items[0];
        }

        function buildDetailForTypeItem(typeItem) {
            detailList.innerHTML = '';
            if (!typeItem) return;

            const typeLink = typeItem.querySelector(':scope > .md-nav__link');
            const typeName = typeLink ? typeLink.textContent.trim() : 'Members';
            detailTitle.innerHTML = '<span class="md-nav__icon md-icon"></span> ' + typeName;

            const childNav = typeItem.querySelector(':scope > .md-nav');
            if (!childNav) return;

            const childItems = childNav.querySelectorAll(':scope > .md-nav__list > .md-nav__item');
            childItems.forEach(function(item) {
                detailList.appendChild(item.cloneNode(true));
            });
        }

        function buildDetailForH2(h2Item) {
            const h3Nav = h2Item.querySelector(':scope > .md-nav');
            const h2Link = h2Item.querySelector(':scope > .md-nav__link');
            const isTypes = isTypesSectionItem(h2Item);

            detailList.innerHTML = '';
            const h2Name = h2Link ? h2Link.textContent.trim() : 'Member functions';
            detailTitle.innerHTML = '<span class="md-nav__icon md-icon"></span> ' + h2Name;

            if (!h3Nav) {
                return;
            }

            if (isTypes) {
                const currentHash = decodeURIComponent(window.location.hash || '');
                const activeTypeItem = findTypeItemForHash(h2Item, currentHash);
                buildDetailForTypeItem(activeTypeItem);
                return;
            }

            // For API pages (api_*, examples_*): show all H3 items (members/functions)
            // These H3 items are hidden in left sidebar, so we clone them for right sidebar
            if (isApiPage) {
                const h3Items = h3Nav.querySelectorAll(':scope > .md-nav__list > .md-nav__item');
                h3Items.forEach(function(h3Item) {
                    detailList.appendChild(h3Item.cloneNode(true));
                });
                return;
            }

            // For non-API pages (e.g., routine_operations), show H4 under the current H3
            // activeH3Item is set by the scroll spy when H3 headings become visible
            if (activeH3Item) {
                const h4Nav = activeH3Item.querySelector(':scope > .md-nav');
                if (h4Nav) {
                    const h4Items = h4Nav.querySelectorAll(':scope > .md-nav__list > .md-nav__item');
                    h4Items.forEach(function(h4Item) {
                        detailList.appendChild(h4Item.cloneNode(true));
                    });
                } else {
                    // H3 has no H4 children, show the H3 itself
                    detailList.appendChild(activeH3Item.cloneNode(true));
                }
            } else {
                // No activeH3Item - show H4 of first H3 (or first H3 if no H4)
                const h3Items = h3Nav.querySelectorAll(':scope > .md-nav__list > .md-nav__item');
                if (h3Items.length > 0) {
                    const firstH3 = h3Items[0];
                    const h4Nav = firstH3.querySelector(':scope > .md-nav');
                    if (h4Nav) {
                        const h4Items = h4Nav.querySelectorAll(':scope > .md-nav__list > .md-nav__item');
                        h4Items.forEach(function(h4Item) {
                            detailList.appendChild(h4Item.cloneNode(true));
                        });
                    } else {
                        detailList.appendChild(firstH3.cloneNode(true));
                    }
                }
            }
        }

        function activateH2Item(h2Item) {
            if (!h2Item) return;
            h2Items.forEach(function(item) {
                item.classList.remove('md-nav__item--active');
            });
            h2Item.classList.add('md-nav__item--active');
            // Reset activeH3Item when switching H2, so buildDetailForH2 will show the current H3's H4
            activeH3Item = null;
            buildDetailForH2(h2Item);
        }

        function findH2ForHash(hash) {
            if (!hash) return null;

            for (const item of h2Items) {
                const h2Link = item.querySelector(':scope > .md-nav__link');
                const h2Hash = normalizeHash(h2Link ? h2Link.getAttribute('href') : '');
                if (h2Hash && h2Hash === hash) {
                    return item;
                }

                const descendants = item.querySelectorAll('a.md-nav__link');
                for (const link of descendants) {
                    const linkHash = normalizeHash(link.getAttribute('href'));
                    if (linkHash && linkHash === hash) {
                        return item;
                    }
                }
            }

            return null;
        }

        function syncFromLocationHash() {
            const currentHash = decodeURIComponent(window.location.hash || '');
            if (!currentHash) return false;

            // First try to find matching H3 within any H2
            for (const h2Item of h2Items) {
                const h3Item = findH3NavItemForHash(h2Item, currentHash);
                if (h3Item) {
                    // Found matching H3, activate its parent H2 first
                    const h2Link = h2Item.querySelector(':scope > .md-nav__link');
                    const h2Hash = normalizeHash(h2Link ? h2Link.getAttribute('href') : '');
                    if (h2Hash) {
                        activateH2Item(h2Item);
                        activeH3Item = h3Item;
                        buildDetailForH2(h2Item);
                        return true;
                    }
                }
            }

            // No matching H3 found, try H2
            const matched = findH2ForHash(currentHash);
            if (matched) {
                activateH2Item(matched);
                return true;
            }
            return false;
        }

        // Detect whether this page is an API reference page (mkdocstrings-generated content).
        // On API pages: H3 members belong to the custom right sidebar only — hide them from the left sidebar.
        // On non-API pages (e.g. routine_operations.md): leave H3 items visible in the integrated left TOC.
        // Use pathname to detect API pages since [data-doc-path] does not exist in generated HTML.
        const isApiPage = /\/(api_|examples_)/.test(window.location.pathname);

        h2Items.forEach(function(h2Item) {
            const h3Nav = h2Item.querySelector(':scope > .md-nav');
            const h2Link = h2Item.querySelector(':scope > .md-nav__link');
            const isTypes = isTypesSectionItem(h2Item);

            if (h2Link) {
                h2Link.addEventListener('click', function() {
                    activateH2Item(h2Item);
                });
            }

            if (h3Nav) {
                // On API pages: hide non-Types H3 in left sidebar (right sidebar handles them).
                // On non-API pages: leave H3 visible in the integrated left TOC.
                h3Nav.style.display = isApiPage && !isTypes ? 'none' : '';
            }
        });

        // Scroll spy: use IntersectionObserver to track both H2 (module) and H3 (member) visibility
        var activeH2Item = null;
        var activeH3Item = null;  // Track active H3 nav item for non-API pages
        var activeH3Id = null;

        function findH3NavItemForHash(h2Item, hash) {
            if (!h2Item) return null;
            const h3Nav = h2Item.querySelector(':scope > .md-nav');
            if (!h3Nav) return null;

            const h3Items = h3Nav.querySelectorAll(':scope > .md-nav__list > .md-nav__item');
            for (const h3Item of h3Items) {
                const h3Link = h3Item.querySelector(':scope > .md-nav__link');
                const h3Hash = normalizeHash(h3Link ? h3Link.getAttribute('href') : '');
                if (h3Hash && h3Hash === hash) {
                    return h3Item;
                }

                // Check descendants for the hash
                const descendants = h3Item.querySelectorAll('a.md-nav__link');
                for (const link of descendants) {
                    const linkHash = normalizeHash(link.getAttribute('href'));
                    if (linkHash && linkHash === hash) {
                        return h3Item;
                    }
                }
            }
            return null;
        }

        function activateH3InDetail(h3Id) {
            if (!detailList) return;
            if (!h3Id) {
                // No specific H3 — clear H3 highlight and reset activeH3Item
                activeH3Item = null;
                Array.from(detailList.querySelectorAll('.md-nav__item--active')).forEach(function(el) {
                    el.classList.remove('md-nav__item--active');
                });
                return;
            }
            if (h3Id === activeH3Id) return;
            activeH3Id = h3Id;

            // Find the nav item whose link href contains the H3 id
            var links = detailList.querySelectorAll('a.md-nav__link');
            var target = null;
            for (var i = 0; i < links.length; i++) {
                var href = links[i].getAttribute('href') || '';
                if (href.indexOf('#' + h3Id) >= 0) {
                    target = links[i];
                    break;
                }
            }

            if (target) {
                var parentItem = target.closest('.md-nav__item');
                if (parentItem) {
                    parentItem.classList.add('md-nav__item--active');
                    // Scroll the right sidebar to bring the active item into view
                    var scrollwrap = rightSidebar ? rightSidebar.querySelector('.md-sidebar__scrollwrap') : null;
                    if (scrollwrap) {
                        var offset = parentItem.offsetTop - scrollwrap.clientHeight / 2;
                        scrollwrap.scrollTop = Math.max(0, offset);
                    }
                }
            }
        }

        function activateH3NavItem(h3Item, h2Item) {
            if (!h3Item) return;
            activeH3Item = h3Item;
            // Use the passed h2Item, or find it if not provided
            if (!h2Item && activeH2Item) {
                for (var i = 0; i < h2Items.length; i++) {
                    var h2Link = h2Items[i].querySelector(':scope > .md-nav__link');
                    var href = h2Link ? h2Link.getAttribute('href') || '' : '';
                    var h2Hash = href;
                    if (h2Hash && h2Hash === '#' + activeH2Item) {
                        h2Item = h2Items[i];
                        break;
                    }
                }
            }
            if (h2Item) {
                buildDetailForH2(h2Item);
            }
        }

        function makeScrollSpyObserver() {
            var observer = new IntersectionObserver(function(entries) {
                var visible = entries.filter(function(e) { return e.isIntersecting; });
                if (visible.length === 0) return;

                // Pick the topmost visible heading (smallest boundingClientRect.top)
                visible.sort(function(a, b) {
                    return a.boundingClientRect.top - b.boundingClientRect.top;
                });
                var best = visible[0].target;

                if (best.tagName === 'H2') {
                    // H2 visible — update left nav (no H3 highlight in right panel yet)
                    if (best.id === activeH2Item) return;
                    var h2Hash = '#' + best.id;
                    var matched = null;
                    for (var i = 0; i < h2Items.length; i++) {
                        var h2Link = h2Items[i].querySelector(':scope > .md-nav__link');
                        var href = h2Link ? h2Link.getAttribute('href') || '' : '';
                        if (href === h2Hash) {
                            matched = h2Items[i];
                            break;
                        }
                    }
                    if (matched) {
                        activeH2Item = best.id;
                        activateH3InDetail(null); // clear H3 highlight
                        activateH2Item(matched);
                    }
                } else if (best.tagName === 'H3') {
                    // H3 visible — activate parent H2 in left nav + highlight H3 in right panel
                    var h3Id = best.id;
                    var parentH2 = best.previousElementSibling;
                    while (parentH2 && parentH2.tagName !== 'H2') {
                        parentH2 = parentH2.previousElementSibling;
                    }
                    if (!parentH2) return;
                    var parentH2Id = parentH2.id;

                    if (parentH2Id !== activeH2Item) {
                        var h2Hash = '#' + parentH2Id;
                        var matchedH2 = null;
                        for (var j = 0; j < h2Items.length; j++) {
                            var h2Link = h2Items[j].querySelector(':scope > .md-nav__link');
                            var href = h2Link ? h2Link.getAttribute('href') || '' : '';
                            if (href === h2Hash) {
                                matchedH2 = h2Items[j];
                                break;
                            }
                        }
                        if (matchedH2) {
                            activeH2Item = parentH2Id;
                            activateH2Item(matchedH2);
                        }
                    }
                    // Find the H3 nav item in the left sidebar and set activeH3Item
                    if (activeH2Item) {
                        for (var k = 0; k < h2Items.length; k++) {
                            var h2Link = h2Items[k].querySelector(':scope > .md-nav__link');
                            var href = h2Link ? h2Link.getAttribute('href') || '' : '';
                            var h2Hash = href;
                            if (h2Hash && h2Hash === '#' + activeH2Item) {
                                var h2Item = h2Items[k];
                                var h3Nav = h2Item.querySelector(':scope > .md-nav');
                                if (h3Nav) {
                                    var h3Items = h3Nav.querySelectorAll(':scope > .md-nav__list > .md-nav__item');
                                    for (var l = 0; l < h3Items.length; l++) {
                                        var h3Item = h3Items[l];
                                        var h3Link = h3Item.querySelector(':scope > .md-nav__link');
                                        var h3Href = h3Link ? h3Link.getAttribute('href') || '' : '';
                                        // Check if this H3 item corresponds to the visible H3
                                        if (h3Href.indexOf('#' + h3Id) >= 0) {
                                            activeH3Item = h3Item;
                                            break;
                                        }
                                    }
                                }
                                break;
                            }
                        }
                    }
                    // Update right sidebar with H4 of current H3
                    if (activeH2Item) {
                        for (var k2 = 0; k2 < h2Items.length; k2++) {
                            var h2Link2 = h2Items[k2].querySelector(':scope > .md-nav__link');
                            var href2 = h2Link2 ? h2Link2.getAttribute('href') || '' : '';
                            var h2Hash2 = href2;
                            if (h2Hash2 && h2Hash2 === '#' + activeH2Item) {
                                var h2Item2 = h2Items[k2];
                                buildDetailForH2(h2Item2);
                                break;
                            }
                        }
                    }
                    activateH3InDetail(h3Id);
                }
            }, {
                rootMargin: '-60px 0px -60% 0px',
                threshold: 0
            });

            // Observe all H2 and H3 headings in the content area
            var headings = document.querySelectorAll('.md-content h2[id], .md-content h3[id]');
            headings.forEach(function(h) { observer.observe(h); });
        }

        // Fallback: sync on hashchange for direct anchor navigation
        window.addEventListener('hashchange', function() {
            syncFromLocationHash();
        });

        // Initial selection: prefer current hash target, fallback to first item with children
        setTimeout(function() {
            if (syncFromLocationHash()) {
                return;
            }

            const firstWithChildren = h2Items.find(function(item) {
                return !!item.querySelector(':scope > .md-nav');
            });

            if (firstWithChildren) {
                activateH2Item(firstWithChildren);
                return;
            }

            if (h2Items.length > 0) {
                activateH2Item(h2Items[0]);
            }
        }, 100);

        // Activate scroll spy
        setTimeout(makeScrollSpyObserver, 200);
    });
})();
