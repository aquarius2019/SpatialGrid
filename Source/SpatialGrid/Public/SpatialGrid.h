// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

SPATIALGRID_API DECLARE_LOG_CATEGORY_EXTERN(LogSpatialGrid, Log, All);

class FSpatialGridModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};