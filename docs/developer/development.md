# Development Guide

## 開発環境のセットアップ
Robopyの開発を始めるには、以下の手順で開発環境をセットアップします。

### リポジトリのクローン

```bash
git clone https://github.com/keio-crl/robopy.git
```

### 依存関係のインストール

```bash
cd robopy
# 開発用依存関係のインストール
uv sync --group dev 
# docs 用依存関係のインストール
uv sync --group docs 
# すべての依存関係のインストール
uv sync --all-groups 
```

## コードスタイル
Robopyでは、コードの一貫性を保つために以下のスタイルガイドラインに従っています。

- **Formatting**: `ruff`をlinterとformatterとして使用しています。コードのフォーマットには`ruff --fix .`を実行してください。
- **Type Checking**: `mypy`を使用して型チェックを行っています。 型チェックには`mypy .`を実行してください。
- **Testing**: `pytest`を使用してユニットテストを実行しています。 テストの実行には`pytest`を実行してください。
