/**
 * Search Focus - Auto-scroll to search result location
 *
 * This script enhances MkDocs Material search to automatically scroll
 * to the specific heading where a match was found.
 */

(function() {
    'use strict';

    // Only apply to API reference pages
    if (!document.querySelector('.md-content__inner')) {
        return;
    }

    // Wait for search results to be rendered
    document.addEventListener('DOMContentLoaded', function() {
        // Monitor for search results
        let searchResultsObserver = null;

        const connectSearchResultsObserver = function() {
            const searchResults = document.querySelector('.md-search-result__list');
            if (!searchResults) {
                return;
            }

            searchResultsObserver = new MutationObserver(function(mutations) {
                mutations.forEach(function(mutation) {
                    if (mutation.addedNodes.length > 0) {
                        mutation.addedNodes.forEach(function(node) {
                            if (node.nodeType === Node.ELEMENT_NODE) {
                                addItemClickListener(node);
                            }
                        });
                    }
                });
            });

            searchResultsObserver.observe(searchResults, { childList: true, subtree: true });
        };

        const addItemClickListener = function(element) {
            // Find all search result items
            const resultItems = element.querySelectorAll('.md-search-result__item');
            resultItems.forEach(function(item) {
                // Skip if already attached
                if (item.hasAttribute('data-search-click-listener')) {
                    return;
                }

                item.setAttribute('data-search-click-listener', 'true');

                // Get the target heading anchor from data attributes
                const heading = item.querySelector('.md-search-result__link');
                if (heading && heading.getAttribute('href')) {
                    heading.addEventListener('click', function(e) {
                        const hash = heading.getAttribute('href');
                        if (hash && hash.startsWith('#')) {
                            // Small delay to let page navigation complete
                            setTimeout(function() {
                                const target = document.querySelector(hash);
                                if (target) {
                                    // Scroll to the element with offset for sticky header
                                    const headerHeight = document.querySelector('header.md-header')?.offsetHeight || 60;
                                    const top = target.getBoundingClientRect().top + window.scrollY - headerHeight - 20;
                                    window.scrollTo({ top: top, behavior: 'smooth' });
                                }
                            }, 100);
                        }
                    });
                }
            });
        };

        connectSearchResultsObserver();

        // Reconnect if search results are re-rendered
        setTimeout(connectSearchResultsObserver, 2000);
        setTimeout(connectSearchResultsObserver, 5000);
    });
})();
