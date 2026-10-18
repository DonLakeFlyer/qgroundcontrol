"""TileJSON and attribution published next to the tiles of each prefix."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from .tilemath import MAX_LATITUDE, MAX_ZOOM

if TYPE_CHECKING:
    from pathlib import Path

TILEJSON_NAME = "tilejson.json"
ATTRIBUTION_NAME = "ATTRIBUTION.txt"

ATTRIBUTION_NOTICES = (
    "Produced using Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus Defence and Space "
    "GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved.",
    "Produced using Copernicus WorldDEM-90 © DLR e.V. 2010-2014 and © Airbus Defence and Space "
    "GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved.",
)


def tilejson(public_url: str, prefix: str) -> dict[str, object]:
    """TileJSON 3.0.0 plus the elevation conventions consumers need (extra keys are allowed)."""
    from . import terrarium

    return {
        "tilejson": "3.0.0",
        "name": "QGroundControl terrain",
        "description": (
            "Copernicus DEM GLO-30 (GLO-90 where GLO-30 is unavailable) as terrarium-encoded "
            "Web Mercator tiles"
        ),
        "attribution": " ".join(ATTRIBUTION_NOTICES),
        "scheme": "xyz",
        "tiles": [f"{public_url}/{prefix}/terrarium/{{z}}/{{x}}/{{y}}.png"],
        "minzoom": 0,
        "maxzoom": MAX_ZOOM,
        "bounds": [-180.0, -MAX_LATITUDE, 180.0, MAX_LATITUDE],
        "encoding": "terrarium",
        "height_step_m": terrarium.HEIGHT_STEP,
        "vertical_datum": "EGM2008",
        "surface": "DSM",
        "sea_level_m": terrarium.SEA_LEVEL,
    }


def write_metadata(out: Path, public_url: str, prefix: str) -> list[Path]:
    """Write the metadata files for *prefix* into *out*; returns their paths."""
    tilejson_path = out / TILEJSON_NAME
    tilejson_path.write_text(
        json.dumps(tilejson(public_url, prefix), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    attribution_path = out / ATTRIBUTION_NAME
    attribution_path.write_text("\n\n".join(ATTRIBUTION_NOTICES) + "\n", encoding="utf-8")
    return [tilejson_path, attribution_path]
