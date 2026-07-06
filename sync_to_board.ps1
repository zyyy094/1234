# 火焰检测项目 - 本地代码同步到远程板子
# 用法: .\sync_to_board.ps1          （手动同步一次）
#       .\sync_to_board.ps1 -Watch   （监听文件变化，自动同步）

param([switch]$Watch)

$REMOTE_HOST = "user@192.168.43.231"
$REMOTE_PATH = "/home/user/fire_detect_project"
$LOCAL_PATH  = $PSScriptRoot

# 同步排除列表
$EXCLUDE = @(
    "build",
    "*.exe",
    "*.out",
    "*.tgz",
    "*.tar.gz",
    "*.zip",
    "fire_detect",
    ".git",
    ".vscode",
    ".idea",
    "__pycache__",
    "*.pyc",
    ".DS_Store",
    "Thumbs.db"
)

function Sync-Code {
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host "同步时间: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor Cyan
    Write-Host "本地: $LOCAL_PATH" -ForegroundColor Cyan
    Write-Host "远程: ${REMOTE_HOST}:${REMOTE_PATH}" -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan

    # 构建 rsync 排除参数
    $excludeArgs = @()
    foreach ($ex in $EXCLUDE) {
        $excludeArgs += "--exclude=$ex"
    }

    # 尝试用 rsync，失败则用 scp
    $rsyncTest = ssh $REMOTE_HOST "which rsync 2>/dev/null" 2>$null
    if ($rsyncTest) {
        Write-Host "[使用 rsync 同步...]" -ForegroundColor Green
        $rsyncCmd = "rsync -avz --progress --delete $($excludeArgs -join ' ') '$LOCAL_PATH/' '${REMOTE_HOST}:${REMOTE_PATH}/'"
        Invoke-Expression $rsyncCmd
        if ($LASTEXITCODE -eq 0) {
            Write-Host "rsync 同步完成!" -ForegroundColor Green
        }
    } else {
        Write-Host "[rsync 不可用，使用 scp 同步源文件...]" -ForegroundColor Yellow
        
        # 获取要同步的源文件
        $sourceFiles = @(
            "*.cpp", "*.h", "CMakeLists.txt", "*.onnx", "*.jpg", ".gitignore"
        )
        
        foreach ($pattern in $sourceFiles) {
            Get-ChildItem $LOCAL_PATH -Filter $pattern -File | ForEach-Object {
                $localFile = $_.FullName
                $remoteFile = "$REMOTE_PATH/$($_.Name)"
                Write-Host "  上传: $($_.Name)" -ForegroundColor Gray
                scp $localFile "${REMOTE_HOST}:${REMOTE_FILE}" 2>$null
            }
        }
        
        # 同步 thirdparty 目录（排除 lib 中的 .so 大文件）
        Write-Host "  同步 thirdparty/ 目录..." -ForegroundColor Gray
        $thirdExclude = @("*.so", "*.tgz", "*.tar.gz")
        $rsyncThird = "rsync -avz --progress $($thirdExclude.ForEach({"--exclude=$_"})) '$LOCAL_PATH/thirdparty/' '${REMOTE_HOST}:${REMOTE_PATH}/thirdparty/'"
        Invoke-Expression $rsyncThird 2>$null
    }
    
    Write-Host ""
}

# 手动同步一次
Sync-Code

# 如果指定了 -Watch，持续监听文件变化
if ($Watch) {
    Write-Host "========================================" -ForegroundColor Magenta
    Write-Host "进入监听模式 - 文件变化时自动同步 (Ctrl+C 退出)" -ForegroundColor Magenta
    Write-Host "========================================" -ForegroundColor Magenta
    
    $watcher = New-Object System.IO.FileSystemWatcher
    $watcher.Path = $LOCAL_PATH
    $watcher.IncludeSubdirectories = $true
    $watcher.EnableRaisingEvents = $true
    $watcher.NotifyFilter = [System.IO.NotifyFilters]::FileName -bor 
                            [System.IO.NotifyFilters]::LastWrite
    
    # 排除的目录
    $watcher.Filter = "*.*"
    
    $lastSync = Get-Date
    
    $action = {
        $path = $Event.SourceEventArgs.FullPath
        $changeType = $Event.SourceEventArgs.ChangeType
        $name = $Event.SourceEventArgs.Name
        
        # 跳过排除的路径
        foreach ($ex in $EXCLUDE) {
            if ($path -match [regex]::Escape($ex)) { return }
        }
        
        # 防抖：2秒内不重复同步
        $elapsed = (Get-Date) - $Event.MessageData
        if ($elapsed.TotalSeconds -lt 2) { return }
        $Event.MessageData = Get-Date
        
        Write-Host "[$changeType] $name" -ForegroundColor DarkGray
    }
    
    Register-ObjectEvent $watcher "Changed" -Action $action -MessageData $lastSync | Out-Null
    Register-ObjectEvent $watcher "Created" -Action $action -MessageData $lastSync | Out-Null
    
    # 注册定时同步（每5秒检查一次，有变化就同步）
    $timer = New-Object System.Timers.Timer
    $timer.Interval = 5000
    $timer.AutoReset = $true
    
    $syncAction = {
        Sync-Code
    }
    Register-ObjectEvent $timer "Elapsed" -Action $syncAction | Out-Null
    $timer.Start()
    
    Write-Host "监听中... 按 Ctrl+C 退出" -ForegroundColor Yellow
    try {
        while ($true) { Start-Sleep -Seconds 1 }
    } finally {
        $timer.Stop()
        $watcher.Dispose()
    }
}
