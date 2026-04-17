// 日本語対応とユーザビリティ向上のためのJavaScript

document.addEventListener('DOMContentLoaded', function() {
    // コードブロックの言語ラベル調整
    const codeBlocks = document.querySelectorAll('pre code[class*="language-"]');
    codeBlocks.forEach(function(block) {
        const className = block.className;
        const language = className.match(/language-(\w+)/);
        if (language) {
            const langName = language[1];
            // 言語名の日本語対応
            const langMap = {
                'python': 'Python',
                'bash': 'Bash',
                'shell': 'Shell',
                'yaml': 'YAML',
                'json': 'JSON',
                'markdown': 'Markdown',
                'javascript': 'JavaScript',
                'typescript': 'TypeScript'
            };
            if (langMap[langName]) {
                // 言語ラベルのカスタマイズ（必要に応じて）
                const pre = block.closest('pre');
                if (pre && !pre.dataset.langProcessed) {
                    pre.dataset.langProcessed = 'true';
                }
            }
        }
    });

    // 外部リンクに新しいタブで開くアイコンを追加
    const externalLinks = document.querySelectorAll('a[href^="http"]:not([href*="' + window.location.hostname + '"])');
    externalLinks.forEach(function(link) {
        link.setAttribute('target', '_blank');
        link.setAttribute('rel', 'noopener noreferrer');

        // 外部リンクアイコンを追加
        if (!link.querySelector('.external-link-icon')) {
            const icon = document.createElement('span');
            icon.className = 'external-link-icon';
            icon.innerHTML = ' ↗';
            icon.style.fontSize = '0.8em';
            icon.style.opacity = '0.7';
            link.appendChild(icon);
        }
    });

    // TOCスクロール位置の調整
    const tocLinks = document.querySelectorAll('.md-nav__link');
    tocLinks.forEach(function(link) {
        link.addEventListener('click', function(e) {
            const targetId = this.getAttribute('href');
            if (targetId && targetId.startsWith('#')) {
                const targetElement = document.querySelector(targetId);
                if (targetElement) {
                    e.preventDefault();
                    setTimeout(function() {
                        targetElement.scrollIntoView({
                            behavior: 'smooth',
                            block: 'start'
                        });
                        // URLを更新
                        window.history.pushState(null, null, targetId);
                    }, 100);
                }
            }
        });
    });

    // コードブロックのコピー機能強化
    const copyButtons = document.querySelectorAll('.md-clipboard');
    copyButtons.forEach(function(button) {
        button.addEventListener('click', function() {
            // コピー成功のフィードバック
            const originalText = button.textContent;
            button.textContent = 'コピーしました！';
            button.style.backgroundColor = '#4caf50';

            setTimeout(function() {
                button.textContent = originalText;
                button.style.backgroundColor = '';
            }, 2000);
        });
    });

    // 検索結果の日本語ハイライト改善
    const searchInput = document.querySelector('.md-search__input');
    if (searchInput) {
        searchInput.addEventListener('input', function() {
            // 日本語入力中の処理（必要に応じて）
            if (this.value.length > 0) {
                // 検索語のハイライト処理
                setTimeout(function() {
                    const results = document.querySelectorAll('.md-search-result__article');
                    results.forEach(function(result) {
                        // 日本語検索結果の強調表示
                        const title = result.querySelector('.md-search-result__title');
                        if (title) {
                            title.style.fontWeight = '600';
                        }
                    });
                }, 100);
            }
        });
    }

    // キーボードショートカットの追加
    document.addEventListener('keydown', function(e) {
        // Ctrl+K or Cmd+K で検索フォーカス
        if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
            e.preventDefault();
            const searchInput = document.querySelector('.md-search__input');
            if (searchInput) {
                searchInput.focus();
            }
        }

        // ESCで検索を閉じる
        if (e.key === 'Escape') {
            const searchInput = document.querySelector('.md-search__input');
            if (searchInput && document.activeElement === searchInput) {
                searchInput.blur();
                const searchForm = document.querySelector('.md-search');
                if (searchForm) {
                    searchForm.classList.remove('md-search--active');
                }
            }
        }
    });

    // ページ読み込み時のスムーズスクロール
    if (window.location.hash) {
        setTimeout(function() {
            const target = document.querySelector(window.location.hash);
            if (target) {
                target.scrollIntoView({
                    behavior: 'smooth',
                    block: 'start'
                });
            }
        }, 300);
    }

    // レスポンシブテーブルの改善
    const tables = document.querySelectorAll('.md-typeset table:not([class])');
    tables.forEach(function(table) {
        const wrapper = document.createElement('div');
        wrapper.className = 'table-wrapper';
        wrapper.style.overflowX = 'auto';
        wrapper.style.marginBottom = '1rem';

        table.parentNode.insertBefore(wrapper, table);
        wrapper.appendChild(table);
    });

    // ダークモード切り替え時の処理
    const themeToggle = document.querySelector('[data-md-component="palette"]');
    if (themeToggle) {
        const observer = new MutationObserver(function(mutations) {
            mutations.forEach(function(mutation) {
                if (mutation.type === 'attributes' && mutation.attributeName === 'data-md-color-scheme') {
                    // ダークモード切り替え時の追加処理
                    console.log('テーマが変更されました:', document.documentElement.getAttribute('data-md-color-scheme'));
                }
            });
        });

        observer.observe(document.documentElement, {
            attributes: true,
            attributeFilter: ['data-md-color-scheme']
        });
    }

    // API ドキュメントの折りたたみ機能
    const apiSections = document.querySelectorAll('.doc-contents');
    apiSections.forEach(function(section) {
        const heading = section.previousElementSibling;
        if (heading && heading.tagName.match(/^H[1-6]$/)) {
            heading.style.cursor = 'pointer';
            heading.addEventListener('click', function() {
                if (section.style.display === 'none') {
                    section.style.display = 'block';
                    heading.textContent = heading.textContent.replace('▶', '▼');
                } else {
                    section.style.display = 'none';
                    heading.textContent = heading.textContent.replace('▼', '▶');
                }
            });
        }
    });
});

// 検索機能の日本語対応強化
if (typeof lunr !== 'undefined') {
    // lunrの日本語対応設定（必要に応じて追加）
    console.log('Lunr検索エンジンが利用可能です');
}

// パフォーマンス監視
if ('performance' in window) {
    window.addEventListener('load', function() {
        setTimeout(function() {
            const perfData = performance.getEntriesByType('navigation')[0];
            if (perfData) {
                console.log('ページ読み込み時間:', Math.round(perfData.loadEventEnd - perfData.fetchStart), 'ms');
            }
        }, 0);
    });
}
