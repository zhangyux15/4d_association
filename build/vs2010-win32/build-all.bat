@echo off

call ..\common\set-global-variables.bat

if "%VS100COMNTOOLS%" == "" (
  msg "%username%" "Visual Studio 10 2010 not detected"
  exit 1
)
if not exist %VSSolutionName%.sln (
  call make-solutions.bat
)
if exist %VSSolutionName%.sln (
  call "%VS100COMNTOOLS%\..\..\VC\vcvarsall.bat"
  MSBuild /property:Configuration="Debug"          %VSSolutionName%.sln
  MSBuild /property:Configuration="MinSizeRel"     %VSSolutionName%.sln
  MSBuild /property:Configuration="Release"        %VSSolutionName%.sln
  MSBuild /property:Configuration="RelWithDebInfo" %VSSolutionName%.sln
)
