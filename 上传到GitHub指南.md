# 如何将项目上传到GitHub

本指南将帮助您将KUKA机器人API项目上传到GitHub。

## 方法一：使用Git命令行（推荐）

### 步骤1：在GitHub上创建新仓库

1. 登录GitHub（如果没有账号，请先注册：https://github.com）
2. 点击右上角的 "+" 号，选择 "New repository"
3. 填写仓库信息：
   - **Repository name**: 例如 `kuka-robot-api` 或 `kuka-lbr-api`
   - **Description**: 例如 "KUKA机器人API控制库 - Python客户端和Java服务器"
   - **Visibility**: 选择 Public（公开）或 Private（私有）
   - **不要**勾选 "Initialize this repository with a README"（因为本地已有文件）
4. 点击 "Create repository"

### 步骤2：初始化本地Git仓库

在项目目录（`C:\Users\Administrator\Desktop\open`）打开命令行（CMD或PowerShell），执行：

```bash
# 初始化Git仓库
git init

# 添加所有文件到暂存区
git add .

# 提交文件
git commit -m "Initial commit: KUKA机器人API控制库"
```

### 步骤3：连接到GitHub远程仓库

```bash
# 添加远程仓库（将 YOUR_USERNAME 和 YOUR_REPO_NAME 替换为实际值）
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git

# 例如：
# git remote add origin https://github.com/yourname/kuka-robot-api.git
```

### 步骤4：推送代码到GitHub

```bash
# 推送到GitHub（首次推送）
git branch -M main
git push -u origin main
```

**注意**：如果这是您第一次使用Git，可能需要配置用户信息：

```bash
git config --global user.name "您的名字"
git config --global user.email "您的邮箱"
```

**如果遇到认证问题**：
- GitHub已不再支持密码认证，需要使用Personal Access Token（个人访问令牌）
- 生成Token：GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
- 创建新token，勾选 `repo` 权限
- 推送时，用户名输入您的GitHub用户名，密码输入生成的token

## 方法二：使用GitHub Desktop（图形界面，更简单）

### 步骤1：下载并安装GitHub Desktop

1. 访问：https://desktop.github.com/
2. 下载并安装GitHub Desktop
3. 登录您的GitHub账号

### 步骤2：在GitHub上创建新仓库

同方法一的步骤1

### 步骤3：使用GitHub Desktop添加仓库

1. 打开GitHub Desktop
2. 点击 "File" → "Add Local Repository"
3. 点击 "Choose..." 选择项目目录：`C:\Users\Administrator\Desktop\open`
4. 如果提示需要初始化，点击 "create a repository"

### 步骤4：提交并推送

1. 在GitHub Desktop中，您会看到所有更改的文件
2. 在左下角填写提交信息，例如："Initial commit: KUKA机器人API控制库"
3. 点击 "Commit to main"
4. 点击右上角的 "Publish repository"
5. 选择仓库名称和可见性（Public/Private）
6. 点击 "Publish Repository"

## 方法三：使用VS Code（如果已安装）

### 步骤1：在VS Code中打开项目

1. 打开VS Code
2. File → Open Folder → 选择项目目录

### 步骤2：初始化Git

1. 点击左侧源代码管理图标（或按 Ctrl+Shift+G）
2. 点击 "Initialize Repository"
3. 点击 "+" 号添加所有文件
4. 填写提交信息并提交

### 步骤3：推送到GitHub

1. 点击左下角的 "..." 菜单
2. 选择 "Remote" → "Add Remote"
3. 输入远程仓库URL：`https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git`
4. 点击 "Publish Branch"

## 后续更新代码

当您修改代码后，需要更新GitHub上的仓库：

### 使用命令行：

```bash
# 查看更改
git status

# 添加更改的文件
git add .

# 提交更改
git commit -m "更新说明：例如：添加新功能或修复bug"

# 推送到GitHub
git push
```

### 使用GitHub Desktop：

1. 在GitHub Desktop中查看更改
2. 填写提交信息
3. 点击 "Commit to main"
4. 点击 "Push origin"

## 常见问题

### Q: 如何删除敏感信息（如IP地址）？

在推送前，检查代码中是否包含：
- 个人IP地址
- 密码或密钥
- 内部网络信息

如果包含，建议：
1. 使用环境变量或配置文件（不提交到Git）
2. 在README中说明如何配置
3. 使用 `.gitignore` 排除配置文件

### Q: 如何添加许可证？

1. 在GitHub仓库页面，点击 "Add file" → "Create new file"
2. 文件名输入 `LICENSE`
3. GitHub会自动提示选择许可证模板
4. 选择MIT、Apache 2.0等常用许可证

### Q: 如何添加项目标签和描述？

在GitHub仓库页面：
1. 点击右侧 "About" 区域
2. 编辑描述和主题标签（Topics）
3. 建议标签：`python`、`kuka`、`robotics`、`api`、`robot-control`

## 完成！

上传成功后，您的项目就可以被其他人看到了（如果是公开仓库）。记得定期更新README和代码！

