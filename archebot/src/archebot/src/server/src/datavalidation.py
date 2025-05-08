from marshmallow import Schema, fields

class LocationLogsSchema(Schema):
    timestamp = fields.DateTime(required=True)
    latitude = fields.Float(required=True)
    longitude = fields.Float(required=True)
    
class LogsSchema(Schema):
    timestamp = fields.DateTime(required=True)
    message = fields.Str(required=True)
    
class RouteSchema(Schema):
    file = fields.Raw(required=True, metadata={'type': 'file', 'format': 'binary'})
    
class CoordinatesSchema(Schema):
    lat = fields.Float(required=True)
    lng = fields.Float(required=True)
    
class BoxCoordinatesSchema(Schema):
    southWest = fields.Nested(CoordinatesSchema, required=True)
    southEast = fields.Nested(CoordinatesSchema, required=True)
    northWest = fields.Nested(CoordinatesSchema, required=True)
    northEast = fields.Nested(CoordinatesSchema, required=True)