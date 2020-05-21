#include "NavMeshCreator.h"
#include "ConfigFile.h"
#include "MeshLoaderObj.h"
#include "BuildContext.h"
NavMeshCreator::NavMeshCreator()
{
	Init();
}

NavMeshCreator::~NavMeshCreator()
{
	Destory();
}

void NavMeshCreator::Create(std::string cfgname,std::string src, std::string dst)
{
	/*printf("config file is %s \n", cfgname.c_str());
	printf("source file is %s \n", src.c_str());
	printf("destination file is %s \n", dst.c_str());*/
	LoadMeshObj(src);
	Build(cfgname);
	Save(dst);
}

bool NavMeshCreator::Init()
{
	m_triareas = NULL;
	_mesh = NULL;
	m_solid = NULL;
	m_chf = NULL;
	m_cset = NULL;
	m_pmesh = NULL;
	m_dmesh = NULL;

	m_filterLowHangingObstacles = true;
	m_filterLedgeSpans = true;
	m_filterWalkableLowHeightSpans = true;
	m_partitionType = SAMPLE_PARTITION_WATERSHED;
	m_keepInterResults = true;
	m_offMeshConCount = 0;

	return true;
}

void NavMeshCreator::Destory()
{
	delete _mesh;
	_mesh = NULL;
	delete[] m_triareas;
	m_triareas = 0;
	rcFreeHeightField(m_solid);
	m_solid = NULL;
	rcFreeCompactHeightfield(m_chf);
	m_chf = NULL;
	rcFreeContourSet(m_cset);
	m_cset = NULL;
	rcFreePolyMesh(m_pmesh);
	m_pmesh = NULL;
	rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = NULL;
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
}

void NavMeshCreator::LoadMeshObj(std::string file)
{
	if (_mesh) 
	{
		delete _mesh;
		_mesh = NULL;
	}
	_mesh = new rcMeshLoaderObj();
	if (!_mesh)
	{
		printf("loadMesh: Out of memory 'm_mesh'.");
		return;
	}
	if (!_mesh->load(file))
	{
		printf("buildTiledNavigation: Could not load '%s'", file.c_str());
		return;
	}
	rcCalcBounds(_mesh->getVerts(), _mesh->getVertCount(), _meshBMin, _meshBMax);

}

void NavMeshCreator::Build(std::string cfgname)
{
	CConfigFile cfgfile;
	if (!cfgfile.Load(cfgname))
	{
		printf("%s not created.\n", cfgname.c_str());
		return;
	}
	//¶ÁÈ¡ÎÄ¼þÅäÖÃ
	float cellsize = cfgfile.GetFloatValue("cellsize");
	float cellheight = cfgfile.GetFloatValue("cellheight");
	float agentheight = cfgfile.GetFloatValue("agentheight");
	float agentradius = cfgfile.GetFloatValue("agentradius");
	float agentmaxclimb = cfgfile.GetFloatValue("agentmaxclimb");
	float agentmaxslope = cfgfile.GetFloatValue("agentmaxslope");
	float regionminsize = cfgfile.GetFloatValue("regionminsize");
	float regionmergesize = cfgfile.GetFloatValue("regionmergesize");
	float edgemaxlen = cfgfile.GetFloatValue("edgemaxlen");
	float edgemaxerror = cfgfile.GetFloatValue("edgemaxerror");
	float vertsperpoly = cfgfile.GetFloatValue("vertsperpoly");
	float detailsampledist = cfgfile.GetFloatValue("detailsampledist");
	float detailsamplemaxerror = cfgfile.GetFloatValue("detailsamplemaxerror");
	float partitiontype = cfgfile.GetFloatValue("partitiontype");
	float tilesize = cfgfile.GetFloatValue("tilesize");
	//meshÅäÖÃ
	const float* bmin = _meshBMin;
	const float* bmax = _meshBMax;
	const float* verts = _mesh->getVerts();
	const int nverts = _mesh->getVertCount();
	const int* tris = _mesh->getTris();
	const int ntris = _mesh->getTriCount();

	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = cellsize;
	m_cfg.ch = cellheight;
	m_cfg.walkableSlopeAngle = agentmaxslope;
	m_cfg.walkableHeight = (int)ceilf(agentheight / m_cfg.ch);
	m_cfg.walkableClimb = (int)floorf(agentmaxclimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(agentradius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(edgemaxlen / cellsize);
	m_cfg.maxSimplificationError = edgemaxerror;
	m_cfg.minRegionArea = (int)rcSqr(regionminsize);
	m_cfg.mergeRegionArea = (int)rcSqr(regionmergesize);
	m_cfg.maxVertsPerPoly = (int)vertsperpoly;
	m_cfg.detailSampleDist = detailsampledist < 0.9f ? 0 : cellsize * detailsampledist;
	m_cfg.detailSampleMaxError = cellsize * detailsamplemaxerror;

	rcVcopy(m_cfg.bmin, bmin);
	rcVcopy(m_cfg.bmax, bmax);
	rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

	BuildContext rccontext;
	BuildContext* m_ctx = &rccontext;

	m_ctx->resetTimers();

	m_ctx->startTimer(RC_TIMER_TOTAL);

	m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
	m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);


	m_solid = rcAllocHeightfield();
	if (!m_solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return;
	}
	if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return;
	}

	m_triareas = new unsigned char[ntris];
	if (!m_triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return;
	}

	memset(m_triareas, 0, ntris * sizeof(unsigned char));
	rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
	if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg.walkableClimb))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return;
	}

	if (!m_keepInterResults)
	{
		delete[] m_triareas;
		m_triareas = 0;
	}

	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);


	m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return;
	}
	if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return;
	}

	if (!m_keepInterResults)
	{
		rcFreeHeightField(m_solid);
		m_solid = 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return;
	}

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		if (!rcBuildDistanceField(m_ctx, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return;
		}

		if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return;
		}
	}

	m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return;
	}
	if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	{
		m_ctx->log(RC_LOG_ERROR,"buildNavigation: Could not create contours.");
		return;
	}

	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return;
	}
	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return;
	}

	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return;
	}

	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}


	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
				m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

			if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}


		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;
		params.offMeshConVerts = m_offMeshConVerts;
		params.offMeshConRad = m_offMeshConRads;
		params.offMeshConDir = m_offMeshConDirs;
		params.offMeshConAreas = m_offMeshConAreas;
		params.offMeshConFlags = m_offMeshConFlags;
		params.offMeshConUserID = m_offMeshConId;
		params.offMeshConCount = m_offMeshConCount;
		params.walkableHeight = agentheight;
		params.walkableRadius = agentradius;
		params.walkableClimb = agentmaxclimb;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return;
		}

		m_navMesh = dtAllocNavMesh();
		if (!m_navMesh)
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return;
		}

		dtStatus status;

		status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return;
		}
	}

	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons \n", m_pmesh->nverts, m_pmesh->npolys);

	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
}


void NavMeshCreator::Save(std::string dst)
{
	if (!m_navMesh) return;
	const dtNavMesh* navmesh = m_navMesh;
	FILE* fp = fopen(dst.c_str(), "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < navmesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = navmesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, navmesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < navmesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = navmesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = navmesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}