from marshmallow import Schema, fields

class LocationLogsSchema(Schema):
    timestamp = fields.DateTime(required=True)
    latitude = fields.Float(required=True)
    longitude = fields.Float(required=True)
    
class LogsSchema(Schema):
    timestamp = fields.DateTime(required=True)
    message = fields.Str(required=True)
    
class ShardsSchema(Schema):
    latitude = fields.Float(required=True)
    longitude = fields.Float(required=True)
    photo = fields.Str(required=False)