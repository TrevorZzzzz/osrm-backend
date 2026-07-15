api_version = 2

Set = require('lib/set')
Sequence = require('lib/sequence')
WayHandlers = require('lib/way_handlers')
find_access_tag = require('lib/access').find_access_tag

function setup()
  local road_speed = 5
  local garden_speed = 4
  local footpath_speed = 3
  local steps_speed = 3
  return {
    properties = {
      weight_name = 'routability',
      max_speed_for_map_matching = 40 / 3.6,
      call_tagless_node_function = false,
      traffic_signal_penalty = 2,
      u_turn_penalty = 2,
      continue_straight_at_waypoint = false,
      use_turn_restrictions = false,
      max_collapse_distance = 10,
      force_split_edges = true
    },
    default_mode = mode.walking,
    default_speed = road_speed,
    oneway_handling = 'specific',
    barrier_blacklist = Set { 'yes', 'wall', 'fence' },
    access_tag_whitelist = Set { 'yes', 'foot', 'permissive', 'designated' },
    access_tag_blacklist = Set { 'no', 'agricultural', 'forestry', 'private', 'delivery', 'use_sidepath' },
    restricted_access_tag_list = Set {},
    restricted_highway_whitelist = Set {},
    construction_whitelist = Set {},
    access_tags_hierarchy = Sequence { 'foot', 'access' },
    service_access_tag_blacklist = Set {},
    restrictions = Sequence { 'foot' },
    suffix_list = Set { 'N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'North', 'South', 'West', 'East' },
    avoid = Set { 'impassable', 'proposed', 'motorroad' },
    speeds = Sequence {
      highway = {
        trunk = road_speed,
        trunk_link = road_speed,
        primary = road_speed,
        primary_link = road_speed,
        secondary = road_speed,
        secondary_link = road_speed,
        tertiary = road_speed,
        tertiary_link = road_speed,
        unclassified = road_speed,
        residential = road_speed,
        living_street = road_speed,
        service = road_speed,
        track = road_speed,
        path = garden_speed,
        pedestrian = garden_speed,
        platform = garden_speed,
        footway = footpath_speed,
        steps = steps_speed,
        pier = garden_speed
      },
      railway = { platform = garden_speed },
      amenity = { parking = road_speed, parking_entrance = road_speed },
      man_made = { pier = garden_speed }
    },
    route_speeds = { ferry = road_speed },
    bridge_speeds = {},
    surface_speeds = {
      fine_gravel = road_speed * 0.75,
      gravel = road_speed * 0.75,
      pebblestone = road_speed * 0.75,
      mud = road_speed * 0.5,
      sand = road_speed * 0.5
    },
    tracktype_speeds = {},
    smoothness_speeds = {}
  }
end

function process_node(profile, node, result)
  local access = find_access_tag(node, profile.access_tags_hierarchy)
  if access then
    if profile.access_tag_blacklist[access] then
      result.barrier = true
    end
  else
    local barrier = node:get_value_by_key('barrier')
    if barrier then
      local bollard = node:get_value_by_key('bollard')
      local rising_bollard = bollard and bollard == 'rising'
      local sensory = node:get_value_by_key('sensory')
      local audible_fence = barrier == 'fence' and sensory and (sensory == 'audible' or sensory == 'audio')
      if profile.barrier_blacklist[barrier] and not rising_bollard and not audible_fence then
        result.barrier = true
      end
    end
  end
  if node:get_value_by_key('highway') == 'traffic_signals' then
    result.traffic_lights = true
  end
end

local function handle_sidewalk_separate(profile, way, result, data)
  local sidewalk = way:get_value_by_key('sidewalk')
  local sidewalk_both = way:get_value_by_key('sidewalk:both')
  local sidewalk_left = way:get_value_by_key('sidewalk:left')
  local sidewalk_right = way:get_value_by_key('sidewalk:right')
  if sidewalk ~= 'separate' and sidewalk_both ~= 'separate' and sidewalk_left ~= 'separate' and sidewalk_right ~= 'separate' then
    return
  end
  if not (data.forward_access and not profile.access_tag_blacklist[data.forward_access]) then
    result.forward_mode = mode.inaccessible
  end
  if not (data.backward_access and not profile.access_tag_blacklist[data.backward_access]) then
    result.backward_mode = mode.inaccessible
  end
  if result.forward_mode == mode.inaccessible and result.backward_mode == mode.inaccessible then
    return false
  end
end

local function apply_base_weight(profile, way, result, data)
  if result.forward_mode ~= mode.inaccessible and result.forward_speed > 0 then
    result.forward_rate = result.forward_speed / 3.6
  end
  if result.backward_mode ~= mode.inaccessible and result.backward_speed > 0 then
    result.backward_rate = result.backward_speed / 3.6
  end
  if result.duration > 0 then
    result.weight = result.duration
  end
end

function process_way(profile, way, result)
  local data = {
    highway = way:get_value_by_key('highway'),
    bridge = way:get_value_by_key('bridge'),
    route = way:get_value_by_key('route'),
    man_made = way:get_value_by_key('man_made'),
    railway = way:get_value_by_key('railway'),
    platform = way:get_value_by_key('platform'),
    amenity = way:get_value_by_key('amenity'),
    public_transport = way:get_value_by_key('public_transport')
  }
  if next(data) == nil then
    return
  end
  local handlers = Sequence {
    WayHandlers.default_mode,
    WayHandlers.blocked_ways,
    WayHandlers.access,
    handle_sidewalk_separate,
    WayHandlers.oneway,
    WayHandlers.destinations,
    WayHandlers.ferries,
    WayHandlers.movables,
    WayHandlers.speed,
    WayHandlers.surface,
    WayHandlers.conveying,
    apply_base_weight,
    WayHandlers.classification,
    WayHandlers.roundabouts,
    WayHandlers.startpoint,
    WayHandlers.names,
    WayHandlers.weights
  }
  WayHandlers.run(profile, way, result, data, handlers)
end

function process_turn(profile, turn)
  turn.duration = 0
  if turn.direction_modifier == direction_modifier.u_turn then
    turn.duration = turn.duration + profile.properties.u_turn_penalty
  end
  if turn.has_traffic_light then
    turn.duration = profile.properties.traffic_signal_penalty
  end
  turn.weight = turn.duration
end

return {
  setup = setup,
  process_way = process_way,
  process_node = process_node,
  process_turn = process_turn
}
