$ErrorActionPreference = 'Stop'
$processor = Get-CimInstance Win32_Processor
$systemInfo = Get-CimInstance Win32_ComputerSystem
$operatingSystem = Get-CimInstance Win32_OperatingSystem
@{ type = 'hardware'; cpu = ($processor.Name | Select-Object -First 1).Trim(); logicalCores = $systemInfo.NumberOfLogicalProcessors; ramMiB = [Math]::Round($systemInfo.TotalPhysicalMemory / 1MB, 2); os = "Windows_NT $($operatingSystem.Version) x64" } | ConvertTo-Json -Compress
while ($null -ne ($request = [Console]::ReadLine())) {
    try {
        $decoded = ConvertFrom-Json -InputObject $request
        $ids = @(foreach ($item in $decoded) { $item })
        if ($ids.Count -eq 0 -or $ids.Count -gt 128 -or ($ids | Where-Object { $_ -isnot [long] -and $_ -isnot [int] -or $_ -le 0 })) { throw 'Invalid process inventory' }
        $processes = @($ids | ForEach-Object { Get-Process -Id $_ -ErrorAction SilentlyContinue })
        $bytes = ($processes | Measure-Object -Property WorkingSet64 -Sum).Sum
        @{ type = 'memory'; requested = $ids.Count; observed = $processes.Count; rssMiB = if ($processes.Count -eq $ids.Count) { $bytes / 1MB } else { $null } } | ConvertTo-Json -Compress
    } catch {
        @{ type = 'memory'; rssMiB = $null; error = $_.Exception.Message } | ConvertTo-Json -Compress
    }
}
