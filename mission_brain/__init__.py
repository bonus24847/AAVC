"""AAVC mission brain — schemas, profile, and the blind-search mission plan.

Competition build: deterministic only (no LLM). The targets are NOT known at
takeoff: ``search_pattern.build_search_pattern`` lays a boustrophedon sweep over
the controlled airspace, and ``live_plan.render_live_plan`` renders the two-stage
MissionPlan that grows a serve pair per target discovered in flight.
"""
