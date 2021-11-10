#!/usr/bin/env python3

import abc
from typing import Dict, Any, Text, NoReturn, Tuple, Optional, List

import sqlite3


class Database:
    """ Sqlite3 database python handler """

    def __init__(self, db_name: Text = 'chat_bot.db'):
        self._db_name = db_name
        self._conn = sqlite3.connect(db_name, 1000, check_same_thread=False)
        self._cur = None
        self._nest_index = 0
        with self as cur:
            cur.execute('PRAGMA foreign_keys = ON')  # enable foreign keys

    @property
    def conn(self) -> sqlite3.Connection:
        return self._conn

    def insert(self, table, data: Dict[Text, Any]) -> NoReturn:
        values = tuple(data.values())
        formatted_keys = ','.join(data.keys())
        with self as cur:
            cur.execute(f'INSERT INTO {table} ({formatted_keys}) VALUES ({"?" + ",?" * (len(values) - 1)})', values)

    def delete(self, table: Text, data: Dict[Text, Any]) -> NoReturn:
        cond = ' AND '.join((f'{key}=?' for key in data.keys()))
        with self as cur:
            cur.execute(f'DELETE FROM {table} WHERE {cond}', tuple(data.values()))

    def update(self, table: Text, set_mapper: Dict[Text, Any], cond_mapper: Dict[Text, Any]) -> NoReturn:
        cond = ' AND '.join((f'{key}=?' for key in cond_mapper.keys()))
        set_keys = ','.join((f'{key}=?' for key in set_mapper.keys()))
        args = tuple(set_mapper.values()) + tuple(cond_mapper.values())
        with self as cur:
            cur.execute(f'UPDATE {table} SET {set_keys} WHERE {cond}', args)

    def search_unique(self, table: str, data: Dict[Text, Any]) -> Any:
        cond = ' AND '.join((f'{key}=?' for key in data.keys()))
        with self as cur:
            return cur.execute(f'SELECT * FROM {table} WHERE {cond}', tuple(data.values())).fetchone()

    def all(self, table: Text) -> Any:
        with self as cur:
            return cur.execute(f'SELECT * FROM {table}').fetchall()

    def __enter__(self) -> sqlite3.Cursor:
        if not self._nest_index:
            self._cur = self._conn.cursor()
        self._nest_index += 1
        return self._cur

    def __exit__(self, *args, **kwargs):
        self._nest_index -= 1
        if not self._nest_index:
            self._conn.commit()
            self._cur.close()
            self._cur = None

    def __del__(self):
        self._conn.close()


class TableBase(abc.ABC):
    def __init__(self, database: Database):
        self.db = database


class ShoppingListTable(TableBase):
    TABLE_NAME = 'ShoppingList'
    DEFAULT_LIST = 'default'

    def __init__(self, database: Database):
        super().__init__(database)
        cmd = f'CREATE TABLE IF NOT EXISTS {self.TABLE_NAME}'
        columns = '(name VARCHAR(64) PRIMARY KEY)'
        with self.db as cur:
            cur.execute(cmd + columns)
            try:
                self.insert(self.DEFAULT_LIST)
            except sqlite3.IntegrityError:
                pass

    def get(self, name: Text) -> Dict[Text, Any]:
        return self._convert_query(self.db.search_unique(self.TABLE_NAME, dict(name=name)))

    def insert(self, name: Text) -> Any:
        with self.db:
            self.db.insert(self.TABLE_NAME, dict(name=name))
            return self.get(name)

    def delete(self, name: Text) -> NoReturn:
        self.db.delete(self.TABLE_NAME, dict(name=name))

    def all(self) -> List[Dict[Text, Any]]:
        data = self.db.all(self.TABLE_NAME)
        return list(map(self._convert_query, data))

    @staticmethod
    def _convert_query(data: Tuple[Text]):
        return dict(name=data[0])


class ItemTable(TableBase):
    TABLE_NAME = 'Item'

    def __init__(self, database: Database):
        super().__init__(database)
        cmd = f'CREATE TABLE IF NOT EXISTS {self.TABLE_NAME}'
        columns = """(
            id VARCHAR(64),
            list_name VARCHAR(64),
            name VARCHAR(64) NOT NULL,
            quantity INT NOT NULL,
            
            PRIMARY KEY (id, list_name),
            FOREIGN KEY (list_name) REFERENCES {} (name) ON DELETE CASCADE ON UPDATE NO ACTION
        )""".format(ShoppingListTable.TABLE_NAME)
        with self.db as cur:
            cur.execute(cmd + columns)

    def get(self, list_name: Text, iid: Text) -> Optional[Dict[Text, Any]]:
        data = self.db.search_unique(self.TABLE_NAME, dict(id=iid, list_name=list_name))
        if data is not None:
            data = self._convert_query(data)
        return data

    def insert(self, list_name: Text, iid: Text, name: Text, quantity: int) -> Dict[Text, Any]:
        with self.db:
            self.db.insert(self.TABLE_NAME, dict(id=iid, list_name=list_name, name=name, quantity=quantity))
            return self.get(list_name, iid)

    def delete(self, list_name: Text, iid: Text) -> NoReturn:
        self.db.delete(self.TABLE_NAME, dict(id=iid, list_name=list_name))

    def update(self, list_name: Text, iid: Text, quantity: int) -> NoReturn:
        self.db.update(self.TABLE_NAME, dict(quantity=quantity), dict(list_name=list_name, id=iid))

    def add_quantity(self, list_name: Text, iid: Text, name: Text, amount: int) -> Dict[Text, Any]:
        with self.db:
            item = self.get(list_name, iid)
            if item is None:
                return self.insert(list_name, iid, name, amount)
            self.update(list_name, iid, item['quantity'] + amount)
            return self.get(list_name, iid)

    def remove_quantity(self, list_name: Text, iid: Text, amount: int) -> Optional[Dict[Text, Any]]:
        with self.db:
            item = self.get(list_name, iid)
            if item is None:
                return None

            new_quantity = item['quantity'] - amount
            if new_quantity <= 0:
                self.delete(list_name, iid)
                return None
            self.update(list_name, iid, new_quantity)
            return self.get(list_name, iid)

    def set_quantity(self, list_name: Text, iid: Text, name: Text, amount: int) -> Any:
        with self.db:
            item = self.get(list_name, iid)
            if item is None:
                return self.insert(list_name, iid, name, amount)
            self.update(list_name, iid, amount)
            return self.get(list_name, iid)

    def all(self) -> List[Dict[Text, Any]]:
        data = self.db.all(self.TABLE_NAME)
        return ShoppingList(list(map(self._convert_query, data)))

    @staticmethod
    def _convert_query(data: Tuple[Text, Text, Text, int]):
        return dict(id=data[0], list_name=data[1], name=data[2], quantity=data[3])

class ShoppingList:
    def __init__(self, data):
        self.data = data
    
    def __str__(self):
        if len(self.data) == 0:
            return "Your shopping list is empty."
        else:
            title = '# -------- SHOPPING LIST  -------- #\n'
            rows = (f'{item["name"]:>18s} - {item["quantity"]}' for item in self.data)
            return title + '\n'.join(rows)
