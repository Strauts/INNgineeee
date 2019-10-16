Add-Type -AssemblyName System.IO.Compression.FileSystem
function Unzip
{
    param([string]$zipfile, [string]$outpath)

    [System.IO.Compression.ZipFile]::ExtractToDirectory($zipfile, $outpath)
}

# find the folder to store the zip file
function Find-Folders {
    [Reflection.Assembly]::LoadWithPartialName("System.Windows.Forms") | Out-Null
    [System.Windows.Forms.Application]::EnableVisualStyles()
    $browse = New-Object System.Windows.Forms.FolderBrowserDialog
    $browse.SelectedPath = "C:\"
    $browse.ShowNewFolderButton = $false
    $browse.Description = "Select a directory for OpenAL"

    $loop = $true
    while($loop)
    {
        if ($browse.ShowDialog() -eq "OK")
        {
        $loop = $false
		
		# extract zip file here
		Unzip "OpenAL.zip" $browse.SelectedPath
		
		#Ask user if he wants to install for all users or current user
		$b = new-object -comobject wscript.shell
		$intAnswer = $a.popup("Do you want to install for all Users? ( No if for current user )", 0,"Set EnvVariable",4)
		# set system environment OPENAL_HOME
		If ($intAnswer -eq 6)
		{
			[Environment]::SetEnvironmentVariable('OPENAL_HOME', "$($browse.SelectedPath)$('\openal-soft-1.19.0-bin')", 'Machine') 
		}
		else
		{
			[Environment]::SetEnvironmentVariable('OPENAL_HOME', "$($browse.SelectedPath)$('\openal-soft-1.19.0-bin')", 'User') 
		}
		#Remove-Item -path "package-zip" -recurse
		Remove-Item -path "OpenAL.zip" -recurse
		
        } else
        {
            $res = [System.Windows.Forms.MessageBox]::Show("You clicked Cancel. Would you like to try again or exit?", "Select a location", [System.Windows.Forms.MessageBoxButtons]::RetryCancel)
            if($res -eq "Cancel")
            {
                #Ends script
                return
            }
        }
    }
    $browse.SelectedPath
    $browse.Dispose()
}

# downloads OpenAL
$invocation = (Get-Variable MyInvocation).Value
$directorypath = Split-Path $invocation.MyCommand.Path
Write-Host $directorypath

#Ask user if he wants to download OpenAL
$a = new-object -comobject wscript.shell
$intAnswer = $a.popup("Do you want to install OpenAL?", `
0,"Install Files",4)
If ($intAnswer -eq 6) {
  Write-Host "Downloading OpenAL..."
	(New-Object Net.WebClient).DownloadFile('http://kcat.strangesoft.net/openal-binaries/openal-soft-1.19.0-bin.zip', 'package-zip')
	Invoke-WebRequest http://kcat.strangesoft.net/openal-binaries/openal-soft-1.19.0-bin.zip -OutFile OpenAL.zip
	Write-Host "Download Complete"
	Find-Folders
}

$intAnswer = $a.popup("Do you want to rename dll files from %OPENAL_HOME%/bin ?", 0, "Rename Files", 4)
  
if($intAnswer -eq 6)
{
	Write-Host "Trying to rename dll files..."
		
	# rename dll files in OPENAL_HOME into OpenAL32.dll
	Rename-Item "$($Env:OPENAL_HOME)\bin\Win32\soft_oal.dll" "OpenAL32.dll"
	Rename-Item "$($Env:OPENAL_HOME)\bin\Win64\soft_oal.dll" "OpenAL32.dll"
}